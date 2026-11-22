"""The PNG encoder thread of a saved recording, against a fake Rerun and fake encoders. No test
waits for real: an encoder's cost moves a FakeClock, and a test decides when a frame may be
encoded with semaphores. A semaphore's timeout only ends a broken test instead of hanging it."""

import contextlib
import io
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from tests.test_rerun_view import ENCODE_S, FakePng, fake_rerun, installed
from kite_bridge import common, rerun_view
from kite_bridge.fake_sim import FakeClock, FakeSim
from kite_bridge.rerun_view import CAMERA_PATH, LIDAR_PATH, PNG_QUEUE_FRAMES, RerunRecorder
from kite_bridge.sim_api import STEP_HZ

COST_S = 0.5
GIVE_UP_S = 10.0


class GatedPng:
    """A fake lossless encoder. Each frame waits for a permit from `allow` before it is encoded,
    then costs COST_S on `clock`. Frame number `fail_at` (counting from 1) raises ValueError
    instead. Returns bytes naming the frame's place in the order encoded."""

    def __init__(self, clock, fail_at=None):
        self.clock, self.fail_at = clock, fail_at
        self.images = []
        self._permits = threading.Semaphore(0)
        self._entered = threading.Semaphore(0)

    def allow(self, frames):
        self._permits.release(frames)

    def wait_until_encoding(self):
        if not self._entered.acquire(timeout=GIVE_UP_S):
            raise AssertionError("the encoder thread never started a frame")

    def __call__(self, pixels):
        self._entered.release()
        if not self._permits.acquire(timeout=GIVE_UP_S):
            raise AssertionError("the test never allowed this frame to be encoded")
        self.images.append(pixels)
        if len(self.images) == self.fail_at:
            raise ValueError("bad pixels")
        self.clock.advance(COST_S)
        return f"png {len(self.images)}".encode()


class LoopClock(FakeClock):
    """A FakeClock that calls `on_loop_read`, once, at the first read by the thread that made it:
    the run's loop. The encoder thread's reads never call it."""

    def __init__(self, on_loop_read):
        super().__init__()
        self._loop = threading.get_ident()
        self._on_loop_read = on_loop_read

    def now(self):
        if self._on_loop_read is not None and threading.get_ident() == self._loop:
            hook, self._on_loop_read = self._on_loop_read, None
            hook()
        return super().now()


def tracked(fake):
    """Makes `fake` note each time set and each log in the order made, as ("time", t) and
    ("log", path, item), with an encoded image as ("png", contents, media type)."""
    events = []
    fake.set_time.side_effect = lambda name, duration: events.append(("time", duration))
    fake.EncodedImage.side_effect = lambda contents, media_type: ("png", contents, media_type)
    fake.Points3D.side_effect = lambda positions, colors: ("points", positions)
    fake.log.side_effect = lambda path, item: events.append(("log", path, item))
    return events


def logged_at(events, path):
    """(time last set before it, item) for each log under `path`, in the order logged."""
    time, found = None, []
    for event in events:
        if event[0] == "time":
            time = event[1]
        elif event[1] == path:
            found.append((time, event[2]))
    return found


def png(n):
    return ("png", f"png {n}".encode(), rerun_view.PNG_MEDIA_TYPE)


def image(n):
    return [[(n, 0, 0)]]


class PngWorkerTest(unittest.TestCase):
    def recorder(self, encoder, clock):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fake = fake_rerun()
        events = tracked(fake)
        with installed(fake):
            recorder = RerunRecorder(Path(tmp.name) / "demo.rrd", arrays=rerun_view.ListArrays(),
                                     encode_png=encoder, clock=clock.now)
        return recorder, fake, events

    def test_every_frame_reaches_the_recording_in_order_at_its_own_time(self):
        clock = FakeClock()
        encoder = FakePng(clock)
        threads_before = set(threading.enumerate())
        recorder, fake, events = self.recorder(encoder, clock)
        for n in range(1, 6):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
            recorder.set_time(n / 10 + 0.05)
            recorder.log_lidar(LIDAR_PATH, [(float(n), 0.0, 0.0)], rerun_view.GROUND_ORIGIN)
        recorder.close()
        self.assertEqual(encoder.images, [image(n) for n in range(1, 6)])
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / 10, png(n)) for n in range(1, 6)])
        self.assertEqual(logged_at(events, LIDAR_PATH),
                         [(n / 10 + 0.05, ("points", [(float(n), 0.0, 0.0)])) for n in range(1, 6)])
        self.assertEqual(recorder.stats.encoded, 5)
        self.assertAlmostEqual(recorder.stats.encode_s, 5 * ENCODE_S)
        fake.disconnect.assert_called_once_with()
        self.assertEqual([t for t in set(threading.enumerate()) - threads_before if t.is_alive()], [])

    def test_a_full_queue_makes_the_loop_wait_and_count_and_drops_nothing(self):
        clock = LoopClock(on_loop_read=lambda: encoder.allow(1))
        encoder = GatedPng(clock)
        recorder, _, events = self.recorder(encoder, clock)
        frames = PNG_QUEUE_FRAMES + 1
        recorder.set_time(0.1)
        recorder.log_image(CAMERA_PATH, image(1))
        encoder.wait_until_encoding()
        for n in range(2, frames + 1):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        # Frame 1 is being encoded and frames 2 to 8 wait behind it: the queue holds 8. Frame 9
        # finds it full, so the loop starts timing its wait, and that first read lets frame 1
        # finish. Frame 1 costs 0.5 fake s, then frame 2 waits for a permit without moving the
        # clock, so the loop waited exactly the 0.5 s of one encode.
        self.assertEqual((recorder.stats.backlog_waits, recorder.stats.backlog_wait_s), (1, COST_S))
        encoder.allow(frames - 1)
        recorder.close()
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / 10, png(n)) for n in range(1, frames + 1)])
        self.assertEqual(encoder.images, [image(n) for n in range(1, frames + 1)])
        self.assertEqual((recorder.stats.encoded, recorder.stats.encode_s), (frames, frames * COST_S))

    def test_frames_encoded_before_a_failure_are_recorded_and_close_reports_the_rest(self):
        clock = FakeClock()
        encoder = GatedPng(clock, fail_at=2)
        recorder, fake, events = self.recorder(encoder, clock)
        for n in range(1, 4):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(3)
        with self.assertRaises(rerun_view.EncodeError) as raised:
            recorder.close()
        # Frame 1 is encoded; frame 2 fails, and frame 3 was queued behind it.
        self.assertEqual(str(raised.exception), "PNG encoding failed on the camera frame at 0.200 s (ValueError: "
                                                "bad pixels); 2 camera frames were not recorded: that one and "
                                                "every frame after it")
        self.assertIsInstance(raised.exception.__cause__, ValueError)
        self.assertEqual(logged_at(events, CAMERA_PATH), [(0.1, png(1))])
        fake.disconnect.assert_called_once_with()


class SavedRunTest(unittest.TestCase):
    """spike_lidar_rerun.py --fake in lock-step with a real RerunRecorder on a fake Rerun, so the
    run goes through `common.session` and its shutdown path."""

    def run_main(self, sim_class, encoder, clock):
        import spike_lidar_rerun

        fake = fake_rerun(writes=1)
        events = tracked(fake)
        sims = []
        closed_before_recording = []
        fake.disconnect.side_effect = lambda: closed_before_recording.extend(s.closed for s in sims)

        def make_sim():
            sims.append(sim_class())
            return sims[-1]

        def make_recorder(save_path, fake_sim, live_stride):
            return RerunRecorder(save_path, live_stride, arrays=rerun_view.ListArrays(), encode_png=encoder,
                                 clock=clock.now)

        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(installed(fake))
            stack.enter_context(mock.patch.object(common, "FakeSim", make_sim))
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", make_recorder))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--fake", "--mode", "lockstep", "--seconds", "1", "--traffic", "0",
                                           "--out", tmp, "--save", "demo.rrd"])
        fake.disconnect.assert_called_once_with()
        self.assertEqual(closed_before_recording, [False])
        self.assertEqual([s.closed for s in sims], [True])
        return code, stderr.getvalue(), events

    def test_ctrl_c_with_frames_queued_records_every_one_then_shuts_down(self):
        clock = FakeClock()
        encoder = GatedPng(clock)
        queued = 5

        class CtrlCWithFramesQueued(FakeSim):
            def step(self, n=1):
                if self._steps == queued:
                    encoder.allow(queued)
                    raise KeyboardInterrupt
                super().step(n)

        code, stderr, events = self.run_main(CtrlCWithFramesQueued, encoder, clock)
        self.assertEqual((code, stderr), (130, "interrupted: BeamNG.tech closed\n"))
        # Steps 1 to 5 each queued one camera frame, and none could be encoded before Ctrl+C in
        # step 6. All five reach the recording in order, each at its step's time.
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / STEP_HZ, png(n)) for n in range(1, queued + 1)])

    def test_an_encoder_exception_is_reported_and_stops_the_run_cleanly(self):
        clock = LoopClock(on_loop_read=lambda: encoder.allow(3))
        encoder = GatedPng(clock, fail_at=3)
        code, stderr, events = self.run_main(FakeSim, encoder, clock)
        # No frame is encoded until the loop finds the queue full: steps 1 to 8 fill it, and step
        # 9 waits. Its wait lets frames 1 and 2 be encoded and frame 3 fail. The run stops in step
        # 9 with frames 3 to 8 queued and frame 9 not yet in, 7 frames not recorded. With the 2
        # recorded, that is all 9 frames of the steps run: none is lost without being counted.
        self.assertEqual(code, 1)
        self.assertEqual(stderr.splitlines()[-1],
                         "error: EncodeError: PNG encoding failed on the camera frame at 0.050 s (ValueError: bad "
                         "pixels); 7 camera frames were not recorded: that one and every frame after it")
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / STEP_HZ, png(n)) for n in (1, 2)])


if __name__ == "__main__":
    unittest.main()

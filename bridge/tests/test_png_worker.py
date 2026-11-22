"""The PNG encoder thread of a saved recording, against a fake Rerun and fake encoders. No test
waits for real: an encoder's cost moves a FakeClock, and a test decides when a frame may be
encoded with semaphores. A semaphore's or a join's timeout only ends a broken test instead of
hanging it. A Ctrl+C that a test sends from inside a fake Rerun call is `signal.raise_signal`,
which runs the SIGINT handler at once, on the run's thread."""

import contextlib
import io
import signal
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from tests.test_rerun_view import ENCODE_S, FakePng, fake_rerun, fake_session, installed, plain_lists
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
    """A FakeClock that runs the next of `hooks` at each read by the thread that made it, the
    run's loop, after reading the time, so that what a hook lets the encoder do cannot move the
    time that read returns. The loop reads this clock only when `put` finds the queue full, at
    the start and at the end of the wait. The encoder thread's reads run no hook."""

    def __init__(self, *hooks):
        super().__init__()
        self._loop = threading.get_ident()
        self._hooks = list(hooks)

    def now(self):
        time = super().now()
        if self._hooks and threading.get_ident() == self._loop:
            self._hooks.pop(0)()
        return time


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
        clock = LoopClock(lambda: encoder.allow(1))
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
        # finds it full, so the loop reads 0 s as its wait starts, and only then lets frame 1
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

    def test_ctrl_c_while_encoded_frames_are_logged_leaves_the_rest_for_close(self):
        clock = FakeClock()
        encoder = GatedPng(clock)
        recorder, fake, events = self.recorder(encoder, clock)
        for n in range(1, 5):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(3)
        # The encoder starts frame 4 only after frames 1 to 3 are encoded, so once it has started
        # four frames, three wait to be logged as one batch at the next set_time.
        for _ in range(4):
            encoder.wait_until_encoding()
        log, camera_logs = fake.log.side_effect, []

        def ctrl_c_at_the_second_camera_log(path, item):
            if path == CAMERA_PATH:
                camera_logs.append(item)
                if len(camera_logs) == 2:
                    raise KeyboardInterrupt
            log(path, item)

        fake.log.side_effect = ctrl_c_at_the_second_camera_log
        stderr = io.StringIO()
        with self.assertRaises(KeyboardInterrupt), contextlib.redirect_stderr(stderr):
            recorder.set_time(0.5)
        encoder.allow(1)
        recorder.close()
        # Frame 1 was logged before Ctrl+C. Frame 2's log raised, so frames 2 and 3 were still
        # waiting, and close logged them and frame 4, each at its own time.
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / 10, png(n)) for n in range(1, 5)])
        fake.disconnect.assert_called_once_with()
        self.assertEqual(stderr.getvalue().splitlines(),
                         ["camera frame at 0.200 s: Rerun's log call raised KeyboardInterrupt; the frame stays queued "
                          "and is logged again, on the rule that a log that raises stored nothing"])

    def test_ctrl_c_as_rerun_takes_a_frame_logs_it_once_and_still_stops_the_loop(self):
        clock = FakeClock()
        encoder = GatedPng(clock)
        recorder, fake, events = self.recorder(encoder, clock)
        for n in range(1, 5):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(3)
        # The encoder starts frame 4 only after frames 1 to 3 are encoded.
        for _ in range(4):
            encoder.wait_until_encoding()
        log = fake.log.side_effect

        def ctrl_c_once_rerun_has_frame_2(path, item):
            log(path, item)
            if item == png(2):
                signal.raise_signal(signal.SIGINT)

        fake.log.side_effect = ctrl_c_once_rerun_has_frame_2
        with self.assertRaises(KeyboardInterrupt):
            recorder.set_time(0.5)
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / 10, png(n)) for n in (1, 2)])
        fake.log.side_effect = log
        encoder.allow(1)
        recorder.close()
        # Rerun had frame 2 when Ctrl+C came, so close logs frames 3 and 4 and never frame 2 again.
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / 10, png(n)) for n in range(1, 5)])

    def test_a_camera_log_that_keeps_raising_still_lets_close_flush_the_recording(self):
        clock = FakeClock()
        encoder = GatedPng(clock)
        recorder, fake, _ = self.recorder(encoder, clock)
        for n in range(1, 3):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(1)
        # The encoder starts frame 2 only after frame 1 is encoded.
        for _ in range(2):
            encoder.wait_until_encoding()
        log = fake.log.side_effect

        def camera_log_fails(path, item):
            if path == CAMERA_PATH:
                raise RuntimeError("camera log failed")
            log(path, item)

        fake.log.side_effect = camera_log_fails
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(RuntimeError):
                recorder.set_time(0.3)
            encoder.allow(1)
            with self.assertRaises(RuntimeError):
                recorder.close()
        fake.disconnect.assert_called_once_with()
        # Frame 1's log raised at set_time and again at close, so neither frame was saved.
        retry = ("camera frame at 0.100 s: Rerun's log call raised RuntimeError; the frame stays queued and is "
                 "logged again, on the rule that a log that raises stored nothing")
        self.assertEqual(stderr.getvalue().splitlines(),
                         [retry, retry, "2 camera frames were not saved: RuntimeError while the recording closed"])

    def test_a_second_ctrl_c_after_the_encoder_failed_names_the_failure_for_the_frames_it_cost(self):
        clock = FakeClock()
        encoder = GatedPng(clock, fail_at=3)
        threads_before = set(threading.enumerate())
        recorder, fake, events = self.recorder(encoder, clock)
        for n in range(1, 5):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(3)
        for thread in set(threading.enumerate()) - threads_before:
            thread.join(GIVE_UP_S)
        log = fake.log.side_effect

        def ctrl_c_once_rerun_has_frame_1(path, item):
            log(path, item)
            if item == png(1):
                signal.raise_signal(signal.SIGINT)

        fake.log.side_effect = ctrl_c_once_rerun_has_frame_1
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(KeyboardInterrupt):
            recorder.close()
        # Frames 1 and 2 were encoded and frame 3 failed, with frame 4 behind it. Frame 1 was
        # logged and then Ctrl+C came, so frame 2 is encoded and not logged, and frames 3 and 4
        # were never encoded: 3 frames not saved, 2 of them because of the encoder.
        self.assertEqual(stderr.getvalue().splitlines(),
                         ["3 camera frames were not saved: 2 because PNG encoding failed on the camera frame at "
                          "0.300 s (ValueError: bad pixels), 1 because Ctrl+C ended the close"])
        self.assertEqual(logged_at(events, CAMERA_PATH), [(0.1, png(1))])
        fake.disconnect.assert_called_once_with()

    def test_a_close_cut_short_by_another_error_after_the_encoder_failed_names_only_the_failure(self):
        clock = FakeClock()
        encoder = GatedPng(clock, fail_at=1)
        threads_before = set(threading.enumerate())
        recorder, fake, _ = self.recorder(encoder, clock)
        for n in range(1, 4):
            recorder.set_time(n / 10)
            recorder.log_image(CAMERA_PATH, image(n))
        encoder.allow(1)
        for thread in set(threading.enumerate()) - threads_before:
            thread.join(GIVE_UP_S)
        fake.disconnect.side_effect = RuntimeError("disconnect failed")
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(RuntimeError):
            recorder.close()
        # Frame 1 failed in the encoder, so frames 1 to 3 were never encoded. The disconnect
        # that raised afterwards did not cost any frame.
        self.assertEqual(stderr.getvalue().splitlines(),
                         ["3 camera frames were not saved: 3 because PNG encoding failed on the camera frame at "
                          "0.100 s (ValueError: bad pixels)"])


class CtrlCHeldTest(unittest.TestCase):
    """`_ctrl_c_held` on a thread that is not the main thread, which never gets a Ctrl+C."""

    def run_off_the_main_thread(self, body):
        outcome = {}

        def work():
            try:
                with rerun_view._ctrl_c_held():
                    body(outcome)
            except BaseException as error:
                outcome["raised"] = error

        thread = threading.Thread(target=work)
        thread.start()
        thread.join(GIVE_UP_S)
        self.assertFalse(thread.is_alive(), "the thread never finished")
        return outcome

    def test_on_the_main_thread_the_block_has_its_own_handler_and_the_old_one_comes_back(self):
        before = signal.getsignal(signal.SIGINT)
        with rerun_view._ctrl_c_held():
            inside = signal.getsignal(signal.SIGINT)
        self.assertNotEqual(inside, before)
        self.assertEqual(signal.getsignal(signal.SIGINT), before)

    def test_off_the_main_thread_the_block_runs_and_leaves_the_handler_alone(self):
        before = signal.getsignal(signal.SIGINT)
        outcome = self.run_off_the_main_thread(
            lambda seen: seen.update(ran=True, inside=signal.getsignal(signal.SIGINT)))
        self.assertEqual(outcome, {"ran": True, "inside": before})
        self.assertEqual(signal.getsignal(signal.SIGINT), before)

    def test_off_the_main_thread_an_error_in_the_block_comes_out_unchanged(self):
        def body(seen):
            raise KeyError("from the block")

        outcome = self.run_off_the_main_thread(body)
        self.assertIsInstance(outcome["raised"], KeyError)
        self.assertEqual(outcome["raised"].args, ("from the block",))


class SavedRunTest(unittest.TestCase):
    """spike_lidar_rerun.py --fake in lock-step with a real RerunRecorder on a fake Rerun, so the
    run goes through `common.session` and its shutdown path."""

    def run_main(self, sim_class, encoder, clock, after_log=None):
        """`after_log(item)`, if given, runs after the fake Rerun has stored each logged item."""
        import spike_lidar_rerun

        fake = fake_rerun(writes=1)
        events = tracked(fake)
        if after_log is not None:
            log = fake.log.side_effect

            def log_then(path, item):
                log(path, item)
                after_log(item)

            fake.log.side_effect = log_then
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

    def test_ctrl_c_in_a_put_waiting_on_a_full_queue_records_that_frame_too_then_shuts_down(self):
        def ctrl_c():
            encoder.allow(PNG_QUEUE_FRAMES + 1)
            raise KeyboardInterrupt

        clock = LoopClock(ctrl_c)
        encoder = GatedPng(clock)
        code, stderr, events = self.run_main(FakeSim, encoder, clock)
        self.assertEqual((code, stderr), (130, "interrupted: BeamNG.tech closed\n"))
        # Steps 1 to 8 fill the queue with frames none of which may be encoded yet. Step 9's put
        # finds it full, and Ctrl+C comes as its wait starts. Frame 9 was the recorder's before
        # the wait, so all 9 frames are encoded at close and reach the recording in order, each
        # at its step's time; run_main checks that the recording closed before the simulator.
        self.assertEqual(logged_at(events, CAMERA_PATH),
                         [(n / STEP_HZ, png(n)) for n in range(1, PNG_QUEUE_FRAMES + 2)])

    def test_a_second_ctrl_c_while_the_recording_closes_says_how_many_frames_were_not_saved(self):
        clock = FakeClock()
        encoder = GatedPng(clock)
        queued = 5
        threads_before = set(threading.enumerate())

        class CtrlCWithFramesQueued(FakeSim):
            def step(self, n=1):
                if self._steps == queued:
                    encoder.allow(2)
                    raise KeyboardInterrupt
                super().step(n)

        def second_ctrl_c(item):
            if item == png(2):
                signal.raise_signal(signal.SIGINT)

        code, stderr, events = self.run_main(CtrlCWithFramesQueued, encoder, clock, after_log=second_ctrl_c)
        encoder.allow(queued - 2)
        for thread in set(threading.enumerate()) - threads_before:
            thread.join(GIVE_UP_S)
        # Steps 1 to 5 queue frames 1 to 5 and Ctrl+C comes in step 6, with only frames 1 and 2
        # allowed to be encoded. Closing logs frame 1, and a second Ctrl+C comes as frame 2 is
        # logged. Frame 3 can never be encoded until the test allows it, so only the second
        # Ctrl+C ends the wait: frames 1 and 2 are saved, and 3, 4 and 5 are counted.
        self.assertEqual((code, stderr.splitlines()),
                         (130, ["3 camera frames were not saved: Ctrl+C while the recording closed",
                                "interrupted: BeamNG.tech closed"]))
        self.assertEqual(logged_at(events, CAMERA_PATH), [(n / STEP_HZ, png(n)) for n in (1, 2)])

    def test_an_encoder_exception_found_by_a_waiting_put_is_reported_and_stops_the_run_cleanly(self):
        clock = LoopClock(lambda: encoder.allow(1))
        encoder = GatedPng(clock, fail_at=1)
        code, stderr, events = self.run_main(FakeSim, encoder, clock)
        # Steps 1 to 8 fill the queue with frames none of which may be encoded yet. Step 9's put
        # queues frame 9 first (R44), finds 9 frames waiting and its wait lets frame 1 be
        # encoded, which fails. A failed frame never leaves the queue, so the queue stays over
        # its bound until the failure ends the wait. The run stops in step 9 with frames 1 to 9
        # queued: 9 frames not recorded, which are all 9 frames of the steps run. Frame 1 was
        # read at 1/60 s.
        self.assertEqual(code, 1)
        self.assertEqual(stderr.splitlines()[-1],
                         "error: EncodeError: PNG encoding failed on the camera frame at 0.017 s (ValueError: bad "
                         "pixels); 9 camera frames were not recorded: that one and every frame after it")
        self.assertEqual(logged_at(events, CAMERA_PATH), [])

    def test_an_encoder_exception_found_by_a_put_that_does_not_wait_is_reported_and_stops_the_run_cleanly(self):
        clock = FakeClock()
        encoder = GatedPng(clock, fail_at=2)
        threads_before = set(threading.enumerate())

        class EncoderFailsBeforeStep4(FakeSim):
            def step(self, n=1):
                if self._steps == 3:
                    encoder.allow(2)
                    for thread in set(threading.enumerate()) - threads_before:
                        thread.join(GIVE_UP_S)
                super().step(n)

        code, stderr, events = self.run_main(EncoderFailsBeforeStep4, encoder, clock)
        # Steps 1 to 3 queue frames 1 to 3. Before step 4 the encoder encodes frame 1, fails on
        # frame 2 and its thread ends. Step 4 logs frame 1, then its put finds 2 frames queued,
        # no wait, and the failure: frames 2 and 3 and step 4's own, 3 frames not recorded. With
        # frame 1 recorded, that is all 4 frames of the steps run. Frame 2 was read at 2/60 s.
        # close does not report the failure a second time, or its count of 2 would replace this.
        self.assertEqual(code, 1)
        self.assertEqual(stderr.splitlines()[-1],
                         "error: EncodeError: PNG encoding failed on the camera frame at 0.033 s (ValueError: bad "
                         "pixels); 3 camera frames were not recorded: that one and every frame after it")
        self.assertEqual(logged_at(events, CAMERA_PATH), [(1 / STEP_HZ, png(1))])

    def test_an_encoder_exception_found_only_at_close_does_not_hide_ctrl_c_or_a_timeout(self):
        stopped_by = [(KeyboardInterrupt, 130, "interrupted: BeamNG.tech closed"),
                      (TimeoutError, 1, f"timed out after {common.DEFAULT_TIMEOUT_S:g} s: BeamNG.tech closed")]
        for stop, exit_code, last_line in stopped_by:
            with self.subTest(stop=stop.__name__):
                clock = FakeClock()
                encoder = GatedPng(clock, fail_at=2)
                threads_before = set(threading.enumerate())
                sims = []

                class EncoderFailsThenStopsInStep4(FakeSim):
                    def step(self, n=1):
                        sims.append(self)
                        if self._steps == 3:
                            encoder.allow(2)
                            for thread in set(threading.enumerate()) - threads_before:
                                thread.join(GIVE_UP_S)
                            raise stop
                        super().step(n)

                code, stderr, events = self.run_main(EncoderFailsThenStopsInStep4, encoder, clock)
                # Steps 1 to 3 queue frames 1 to 3. In step 4 the encoder encodes frame 1, fails
                # on frame 2 and its thread ends, and then the step stops the run before any put
                # can find the failure. close logs frame 1 and reports frames 2 and 3 as not
                # recorded. Frame 2 was read at 2/60 s.
                self.assertEqual(code, exit_code)
                self.assertEqual(stderr.splitlines(),
                                 ["error: EncodeError: PNG encoding failed on the camera frame at 0.033 s (ValueError: "
                                  "bad pixels); 2 camera frames were not recorded: that one and every frame after it",
                                  last_line])
                self.assertEqual(logged_at(events, CAMERA_PATH), [(1 / STEP_HZ, png(1))])
                self.assertFalse(sims[-1].sensors_removed)

    def test_a_saved_run_prints_the_encoding_time_and_the_backlog_waits(self):
        import spike_lidar_rerun

        clock = LoopClock(lambda: encoder.allow(1), lambda: encoder.allow(PNG_QUEUE_FRAMES))
        encoder = GatedPng(clock)
        stdout, stderr = io.StringIO(), io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(installed(fake_rerun(writes=1)))
            stack.enter_context(plain_lists(encoder))
            stack.enter_context(mock.patch.object(common, "session", fake_session))
            stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--home", tmp, "--out", tmp, "--mode", "lockstep", "--seconds", "0.15",
                                           "--traffic", "0", "--save", "demo.rrd"])
        self.assertEqual(code, 0, stderr.getvalue())
        # 0.15 s of lock-step is 9 steps, one camera frame each. Steps 1 to 8 fill the queue with
        # frames none of which may be encoded yet, and step 9 finds it full: one wait. The wait
        # starts at 0 s and lets frame 1 be encoded, 0.5 fake s. Frame 2 then waits for a permit,
        # so the wait ends at 0.5 s, and only that end lets the other 8 frames be encoded. The 9
        # frames at 0.5 fake s each took 4.5 s.
        self.assertIn("PNG encoding: 9 camera images in 4.500 s on the encoder thread", stdout.getvalue())
        self.assertIn("PNG backlog: the loop found the queue of 8 frames full 1 times and waited 0.500 s in all "
                      "for space; no frame was dropped", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()

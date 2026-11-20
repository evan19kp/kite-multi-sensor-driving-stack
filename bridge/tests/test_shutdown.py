import _thread
import contextlib
import io
import json
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from tests.test_car_and_versions import LOCKSTEP_ARGS
from tests.test_free_running import BoundedClock
from tests.test_release import Sensor
from tests.test_rerun_view import RecordingRecorder
from tests.test_slow_polls import RawRecorder
from kite_bridge import common
from kite_bridge.fake_sim import FakeSim
from kite_bridge.sim_api import remove_all

LIMIT_S = 5.0
SCRIPTS = ("spike_connect", "spike_sensors", "spike_multicar", "spike_lidar_rerun")


class InterruptedStep(FakeSim):
    def step(self, n=1):
        raise KeyboardInterrupt


class TimedOutStep(FakeSim):
    def step(self, n=1):
        raise socket.timeout("timed out")


class TimedOutRemoval(FakeSim):
    def remove_sensors(self):
        raise socket.timeout("timed out")


class FailingClose(FakeSim):
    def close(self):
        super().close()
        raise RuntimeError("close failed")


class InterruptedStepFailingClose(InterruptedStep, FailingClose):
    pass


class NotLaunched(InterruptedStepFailingClose):
    def end_process(self):
        return False


class InterruptedRemoval(FakeSim):
    def remove_sensors(self):
        raise KeyboardInterrupt


class FailingRemovalAndClose(FailingClose):
    def remove_sensors(self):
        raise OSError("busy")


class ShutdownTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name)

    def run_main(self, sim_class, *argv, script="spike_sensors", patches=()):
        module = __import__(script)
        made = []

        def make():
            made.append(sim_class())
            return made[-1]

        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(common, "FakeSim", make))
            for patch in patches:
                stack.enter_context(patch)
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            started = time.monotonic()
            code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(self.out),
                                *LOCKSTEP_ARGS.get(script, ()), *argv])
            elapsed = time.monotonic() - started
        self.assertLess(elapsed, LIMIT_S)
        return code, stderr.getvalue().splitlines(), made

    def test_a_clean_run_removes_the_sensors_closes_and_ends_no_process(self):
        code, stderr, (sim,) = self.run_main(FakeSim)
        self.assertEqual((code, stderr), (0, []))
        self.assertEqual((sim.sensors_removed, sim.closed, sim.process_ended), (True, True, False))

    def test_ctrl_c_in_step_skips_sensor_removal_closes_and_exits_130(self):
        code, stderr, (sim,) = self.run_main(InterruptedStep)
        self.assertEqual(code, 130)
        self.assertEqual(stderr, ["interrupted: BeamNG.tech closed"])
        self.assertEqual((sim.sensors_removed, sim.closed, sim.process_ended), (False, True, False))

    def test_every_script_shuts_down_the_same_way_on_ctrl_c(self):
        for script in SCRIPTS:
            code, stderr, sims = self.run_main(InterruptedStep, script=script)
            self.assertEqual((code, stderr), (130, ["interrupted: BeamNG.tech closed"]), script)
            self.assertEqual([(s.sensors_removed, s.closed) for s in sims], [(False, True)], script)

    def test_a_timeout_in_step_skips_sensor_removal_and_names_the_timeout(self):
        code, stderr, (sim,) = self.run_main(TimedOutStep, "--timeout", "2.5")
        self.assertEqual(code, 1)
        self.assertEqual(stderr, ["timed out after 2.5 s: BeamNG.tech closed"])
        self.assertEqual((sim.sensors_removed, sim.closed, sim.process_ended), (False, True, False))

    def test_a_poll_that_times_out_stops_the_run_instead_of_counting_as_unexpected(self):
        class TimedOutPoll(FakeSim):
            def poll(self, car, kind):
                raise socket.timeout("timed out")

        code, stderr, (sim,) = self.run_main(TimedOutPoll)
        self.assertEqual((code, stderr), (1, ["timed out after 300 s: BeamNG.tech closed"]))
        self.assertFalse((self.out / "sensors.json").exists())
        self.assertTrue(sim.closed)

    def test_a_sensor_removal_that_times_out_still_closes_and_keeps_the_results(self):
        code, stderr, (sim,) = self.run_main(TimedOutRemoval)
        self.assertEqual(code, 1)
        self.assertEqual(stderr, ["timed out after 300 s: BeamNG.tech closed"])
        self.assertEqual((sim.closed, sim.process_ended), (True, False))
        self.assertTrue((self.out / "sensors.json").is_file())

    def test_a_removal_timeout_after_a_run_error_is_printed_and_the_run_error_is_reported(self):
        class FailingStepTimedOutSensor(FakeSim):
            def step(self, n=1):
                raise RuntimeError("step failed")

            def remove_sensors(self):
                remove_all({("kite1", "lidar"): Sensor(socket.timeout("timed out"))})

        code, stderr, (sim,) = self.run_main(FailingStepTimedOutSensor)
        self.assertEqual(code, 1)
        self.assertEqual(stderr[0], "warning: could not remove kite1 lidar: TimeoutError: timed out")
        self.assertIn("Traceback (most recent call last):", stderr)
        self.assertEqual(stderr[-1], "error: RuntimeError: step failed")
        self.assertEqual((sim.closed, sim.process_ended), (True, False))

    def test_a_traffic_spawn_that_times_out_stops_the_run(self):
        class TimedOutTraffic(FakeSim):
            def spawn_traffic(self, n):
                raise socket.timeout("timed out")

        code, stderr, (sim,) = self.run_main(TimedOutTraffic, "--traffic", "2")
        self.assertEqual((code, stderr), (1, ["timed out after 300 s: BeamNG.tech closed"]))
        self.assertEqual((sim.sensors_removed, sim.closed), (False, True))

    def test_reading_the_other_cars_positions_that_times_out_stops_the_run(self):
        class TimedOutOthers(FakeSim):
            def other_positions(self):
                raise socket.timeout("timed out")

        code, stderr, (sim,) = self.run_main(TimedOutOthers, "--traffic", "2", script="spike_lidar_rerun")
        self.assertEqual((code, stderr), (1, ["timed out after 300 s: BeamNG.tech closed"]))
        self.assertEqual((sim.sensors_removed, sim.closed), (False, True))

    def test_a_close_that_raises_ends_the_process_and_reports_the_error(self):
        code, stderr, (sim,) = self.run_main(FailingClose)
        self.assertEqual(code, 1)
        self.assertIn("BeamNG.tech not closed (RuntimeError: close failed); ended the process BeamNGpy launched",
                      stderr)
        self.assertIn("Traceback (most recent call last):", stderr)
        self.assertEqual(stderr[-1], "error: RuntimeError: close failed")
        self.assertEqual((sim.sensors_removed, sim.process_ended), (True, True))

    def test_a_close_failure_is_reported_even_after_a_removal_failure(self):
        code, stderr, (sim,) = self.run_main(FailingRemovalAndClose)
        self.assertEqual(code, 1)
        self.assertEqual(stderr[-1], "error: RuntimeError: close failed")
        self.assertTrue(sim.process_ended)

    def test_ctrl_c_then_a_failing_close_ends_the_process_and_exits_130(self):
        code, stderr, (sim,) = self.run_main(InterruptedStepFailingClose)
        self.assertEqual(code, 130)
        self.assertEqual(stderr, [
            "BeamNG.tech not closed (RuntimeError: close failed); ended the process BeamNGpy launched",
            "interrupted: BeamNG.tech closed"])
        self.assertEqual((sim.sensors_removed, sim.process_ended), (False, True))

    def test_a_simulator_beamngpy_did_not_launch_is_never_ended_and_the_run_says_so(self):
        code, stderr, _ = self.run_main(NotLaunched)
        self.assertEqual(code, 130)
        self.assertEqual(stderr, [
            "BeamNG.tech not closed (RuntimeError: close failed); BeamNGpy did not launch it, so nothing "
            "was ended: close BeamNG.tech by hand",
            "interrupted: BeamNG.tech may still be running"])

    def test_a_close_that_never_returns_ends_the_process_after_the_close_wait(self):
        release = threading.Event()
        self.addCleanup(release.set)

        class HangingClose(FakeSim):
            def close(self):
                release.wait(LIMIT_S)

        # 0.2 s stands in for the real 20 s wait, so the test never waits long.
        wait = mock.patch.object(common, "CLOSE_WAIT_S", 0.2)
        code, stderr, (sim,) = self.run_main(HangingClose, patches=[wait])
        self.assertEqual(code, 1)
        self.assertEqual(stderr, [
            "BeamNG.tech not closed (close did not return within 0.2 s); ended the process BeamNGpy launched",
            "error: close did not return within 0.2 s"])
        self.assertTrue(sim.process_ended)

    def test_a_second_ctrl_c_during_sensor_removal_skips_close_and_ends_the_process(self):
        code, stderr, (sim,) = self.run_main(InterruptedRemoval)
        self.assertEqual(code, 130)
        self.assertEqual(stderr, [
            "BeamNG.tech not closed (Ctrl+C during shutdown); ended the process BeamNGpy launched",
            "interrupted: BeamNG.tech closed"])
        self.assertEqual((sim.closed, sim.process_ended), (False, True))

    def test_a_second_ctrl_c_while_waiting_for_close_ends_the_process_at_once(self):
        release = threading.Event()
        self.addCleanup(release.set)

        class CtrlCDuringClose(InterruptedStep):
            def close(self):
                _thread.interrupt_main()
                release.wait(LIMIT_S)

        # The 4 s wait is long enough that only the second Ctrl+C can end the wait within 2 s.
        wait = mock.patch.object(common, "CLOSE_WAIT_S", 4.0)
        started = time.monotonic()
        code, stderr, (sim,) = self.run_main(CtrlCDuringClose, patches=[wait])
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertEqual(code, 130)
        self.assertEqual(stderr, [
            "BeamNG.tech not closed (Ctrl+C during shutdown); ended the process BeamNGpy launched",
            "interrupted: BeamNG.tech closed"])
        self.assertTrue(sim.process_ended)

    def test_ctrl_c_in_the_second_multicar_case_keeps_the_first_case(self):
        class SecondCaseInterrupted(FakeSim):
            cases = 0

            def load_scenario(self, model, n_cars, config=None):
                SecondCaseInterrupted.cases += 1
                if SecondCaseInterrupted.cases == 2:
                    raise KeyboardInterrupt
                return super().load_scenario(model, n_cars, config)

        code, stderr, sims = self.run_main(SecondCaseInterrupted, script="spike_multicar")
        self.assertEqual((code, stderr), (130, ["interrupted: BeamNG.tech closed"]))
        self.assertEqual([s.closed for s in sims], [True, True])
        saved = json.loads((self.out / "multicar.json").read_text(encoding="utf-8"))
        self.assertEqual([c["cars"] for c in saved["cases"]], [1])


class RecordingAfterCtrlCTest(unittest.TestCase):
    def test_ctrl_c_closes_the_recording_with_every_step_logged_before_it(self):
        import spike_lidar_rerun

        class CtrlCAtStep31(FakeSim):
            def step(self, n=1):
                if self._steps == 30:
                    raise KeyboardInterrupt
                super().step(n)

        recorder = RecordingRecorder()
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", lambda path, fake: recorder))
            stack.enter_context(mock.patch.object(common, "FakeSim", CtrlCAtStep31))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            started = time.monotonic()
            code = spike_lidar_rerun.main(["--fake", "--mode", "lockstep", "--seconds", "1", "--traffic", "0",
                                           "--out", tmp, "--save", "demo.rrd"])
        self.assertLess(time.monotonic() - started, LIMIT_S)
        self.assertEqual((code, stderr.getvalue()), (130, "interrupted: BeamNG.tech closed\n"))
        self.assertEqual(recorder.closed, 1)
        self.assertEqual(len([c for c in recorder.calls if c[0] == "time"]), 30)

    def test_ctrl_c_in_the_default_free_run_closes_the_recording_before_beamng(self):
        import spike_lidar_rerun

        for method, call in (("stream", 5), ("timer_seconds", 2)):
            with self.subTest(method=method):
                clock = BoundedClock()
                sims = []
                recorder = ClosingOrderRecorder(sims)

                def make():
                    sims.append(CtrlCOnCall(method, call, clock=clock))
                    return sims[-1]

                stderr = io.StringIO()
                with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
                    stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder",
                                                          lambda path, fake: recorder))
                    stack.enter_context(mock.patch.object(common, "FakeSim", make))
                    stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
                    stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
                    stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                    stack.enter_context(contextlib.redirect_stderr(stderr))
                    code = spike_lidar_rerun.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", tmp,
                                                   "--save", "demo.rrd"])
                self.assertEqual((code, stderr.getvalue()), (130, "interrupted: BeamNG.tech closed\n"))
                self.assertEqual(recorder.closed, 1)
                self.assertEqual(recorder.sims_closed_when_closed, [False])
                self.assertEqual([(s.sensors_removed, s.closed) for s in sims], [(False, True)])
                self.assertEqual(sims[0].calls[method], call)
                if method == "stream":
                    # Every streamed read before the interrupted one gave a lidar scan or a camera
                    # image with a colour image, and each was logged.
                    logged = [c for c in recorder.calls if c[0] in ("points", "image")]
                    self.assertEqual(len(logged), call - 1)


class CtrlCOnCall(FakeSim):
    """Raises KeyboardInterrupt on call number `call` of `method` ("stream" or "timer_seconds")."""

    def __init__(self, method, call, **kwargs):
        super().__init__(**kwargs)
        self.method, self.call = method, call
        self.calls = {"stream": 0, "timer_seconds": 0}

    def _count(self, method):
        self.calls[method] += 1
        if method == self.method and self.calls[method] == self.call:
            raise KeyboardInterrupt

    def stream(self, car, kind):
        self._count("stream")
        return super().stream(car, kind)

    def timer_seconds(self):
        self._count("timer_seconds")
        return super().timer_seconds()


class ClosingOrderRecorder(RawRecorder):
    """Notes, when the recording closes, whether each simulator was already closed."""

    def __init__(self, sims):
        super().__init__()
        self.sims = sims
        self.sims_closed_when_closed = None

    def close(self):
        super().close()
        self.sims_closed_when_closed = [s.closed for s in self.sims]


def run_script(name, *args):
    done = subprocess.run([sys.executable, str(BRIDGE_DIR / f"{name}.py"), *args],
                          capture_output=True, text=True, timeout=120)
    return done.returncode, done.stdout, done.stderr


class TimeoutOptionTest(unittest.TestCase):
    def test_a_timeout_that_is_not_above_zero_is_a_usage_error(self):
        for script in SCRIPTS:
            code, stdout, stderr = run_script(script, "--fake", "--timeout", "0")
            self.assertEqual((code, stdout), (2, ""), script)
            self.assertIn("--timeout", stderr)
        for bad in ("-1", "nan", "x"):
            code, _, stderr = run_script("spike_connect", "--fake", "--timeout", bad)
            self.assertEqual(code, 2, bad)
            self.assertIn("usage", stderr)

    def test_help_lists_the_timeout_and_its_default(self):
        for script in SCRIPTS:
            code, stdout, _ = run_script(script, "--help")
            flat = " ".join(stdout.split())
            self.assertEqual(code, 0)
            self.assertIn("--timeout SECONDS", flat)
            self.assertIn("timed out (default 300)", flat)


if __name__ == "__main__":
    unittest.main()

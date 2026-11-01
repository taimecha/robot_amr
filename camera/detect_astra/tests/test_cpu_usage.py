from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cpu_usage import CpuMonitor, cpu_delta


class CpuUsageTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.monitor = CpuMonitor(proc_root=self.root, start=False)

    def tearDown(self):
        self.monitor.close()
        self.directory.cleanup()

    def write_cpus(self, step):
        core = [10 + step * 40, 0, 10 + step * 20, 80 + step * 40, 0, 0, 0, 0]
        lines = ["cpu " + " ".join(str(v * 4) for v in core)]
        lines += [f"cpu{i} " + " ".join(map(str, core)) for i in range(4)]
        (self.root / "stat").write_text("\n".join(lines))

    def write_process(self, pid=123, ticks=100, started=999, script="publish_pallet_target.py"):
        entry = self.root / str(pid)
        entry.mkdir(exist_ok=True)
        (entry / "cmdline").write_bytes(b"python\0" + script.encode() + b"\0")
        fields = ["S"] + ["0"] * 49
        fields[11], fields[19] = str(ticks), str(started)
        (entry / "stat").write_text(f"{pid} (process (name) with spaces) " + " ".join(fields))

    def test_whole_machine_and_per_core_normalization(self):
        self.write_cpus(0)
        self.write_process(ticks=100)
        self.monitor.sample()
        self.assertIsNone(self.monitor.snapshot()["total_percent"])
        self.write_cpus(1)
        self.write_process(ticks=300)
        self.monitor.sample()
        cpu = self.monitor.snapshot()
        self.assertEqual(cpu["core_count"], 4)
        self.assertEqual(cpu["total_percent"], 60.0)
        self.assertEqual([c["percent"] for c in cpu["cores"]], [60.0] * 4)
        self.assertEqual(cpu["recognition_percent"], 50.0)
        self.assertEqual(cpu["recognition_cores"], 2.0)

        self.write_cpus(2)
        self.write_process(ticks=5, started=1000)  # reused PID must not create a spike
        self.monitor.sample()
        self.assertIsNone(self.monitor.snapshot()["recognition_percent"])
        (self.root / "123/stat").unlink()
        self.write_cpus(3)
        self.monitor.sample()
        self.assertEqual(self.monitor.snapshot()["pids"], [])
        self.assertIsNone(self.monitor.snapshot()["recognition_percent"])

    def test_process_discovery_ignores_shell_command_text(self):
        self.write_process()
        self.write_process(pid=456, script="bash -c publish_pallet_target.py")
        self.assertEqual(self.monitor.read_processes(), {(123, 999): 100})

    def test_guest_time_is_not_counted_twice(self):
        previous = [100, 0, 0, 100, 0, 0, 0, 0, 10, 0]
        current = [140, 0, 0, 160, 0, 0, 0, 0, 30, 0]
        self.assertEqual(cpu_delta(previous, current), (100, 40.0))

    def test_detector_and_tracker_are_both_counted(self):
        self.write_cpus(0)
        self.write_process(pid=123, ticks=100)
        self.write_process(pid=456, ticks=100, script='publish_pallet_detection.py')
        self.monitor.sample()
        self.write_cpus(1)
        self.write_process(pid=123, ticks=140)
        self.write_process(pid=456, ticks=220, script='publish_pallet_detection.py')
        self.monitor.sample()
        self.assertEqual(self.monitor.snapshot()['recognition_percent'], 40.0)
        self.assertEqual(sorted(self.monitor.snapshot()['pids']), [123, 456])


if __name__ == "__main__":
    unittest.main()

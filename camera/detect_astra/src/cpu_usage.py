"""Short-interval Linux CPU usage, normalized to the whole machine."""
import os
from pathlib import Path
import threading


def cpu_delta(previous, current):
    # user/nice already include guest time. Count only the first eight fields.
    delta = [max(0, b - a) for a, b in zip(previous[:8], current[:8])]
    total = sum(delta)
    idle = sum(delta[3:5])  # idle + iowait
    return total, 100.0 * (total - idle) / total if total else None


class CpuMonitor:
    def __init__(self, recognition_pid=None, proc_root=Path("/proc"), start=True):
        self.recognition_pid = recognition_pid
        self.proc_root = Path(proc_root)
        self.previous = None
        self.result = {"total_percent": None, "cores": [], "core_count": os.cpu_count() or 1,
                       "recognition_percent": None, "recognition_cores": None, "pids": [],
                       "error": None}
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True) if start else None
        if self.thread:
            self.thread.start()

    def read_processes(self):
        processes = {}
        entries = ([self.proc_root / str(self.recognition_pid)] if self.recognition_pid is not None
                   else self.proc_root.iterdir())
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                if self.recognition_pid is None:
                    if entry.stat().st_uid != os.getuid():
                        continue
                    argv = (entry / "cmdline").read_bytes().split(b"\0")
                    if not any(Path(os.fsdecode(arg)).name in
                               ("publish_pallet_target.py", "publish_pallet_detection.py")
                               for arg in argv if arg):
                        continue
                # comm can contain spaces and parentheses; fields after it start at state (3).
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                identity = (int(entry.name), int(fields[19]))  # PID + process start time
                processes[identity] = int(fields[11]) + int(fields[12])  # utime + stime
            except (OSError, ValueError, IndexError):
                continue
        return processes

    def sample(self):
        cpus = {}
        for line in (self.proc_root / "stat").read_text().splitlines():
            fields = line.split()
            if fields and (fields[0] == "cpu" or fields[0][3:].isdigit() and fields[0].startswith("cpu")):
                cpus[fields[0]] = [int(v) for v in fields[1:9]]
        processes = self.read_processes()
        count = len(cpus) - 1
        total_percent = process_percent = process_cores = None
        cores = [{"name": name, "percent": None} for name in cpus if name != "cpu"]
        if self.previous:
            old_cpus, old_processes = self.previous
            total_ticks, total_percent = cpu_delta(old_cpus["cpu"], cpus["cpu"])
            cores = [{"name": name, "percent": cpu_delta(old_cpus[name], values)[1]
                      if name in old_cpus else None}
                     for name, values in cpus.items() if name != "cpu"]
            common = processes.keys() & old_processes.keys()
            if total_ticks and common:
                ticks = sum(max(0, processes[key] - old_processes[key]) for key in common)
                process_percent = min(100.0, ticks / total_ticks * 100.0)
                process_cores = process_percent * count / 100.0
        self.previous = cpus, processes
        result = {"total_percent": total_percent, "cores": cores, "core_count": count,
                  "recognition_percent": process_percent, "recognition_cores": process_cores,
                  "pids": [pid for pid, _ in processes], "error": None}
        with self.lock:
            self.result = result

    def run(self):
        while not self.stopped.is_set():
            try:
                self.sample()
            except (OSError, ValueError, KeyError):
                with self.lock:
                    self.result = {**self.result, "total_percent": None, "cores": [],
                                   "recognition_percent": None, "recognition_cores": None,
                                   "error": "Không đọc được tải CPU"}
            if self.stopped.wait(1.0):
                break

    def snapshot(self):
        with self.lock:
            return dict(self.result)

    def close(self):
        self.stopped.set()
        if self.thread:
            self.thread.join()

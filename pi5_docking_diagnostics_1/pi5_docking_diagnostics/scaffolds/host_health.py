#!/usr/bin/env python3
"""Read-only Linux/Pi sampler. No ROS, sockets or actuator access. JSON lines."""
import argparse
import json
import pathlib
import shutil
import subprocess
import time


def read(path):
    try:
        return pathlib.Path(path).read_text().strip()
    except OSError:
        return None


def cpu_ticks():
    result = {}
    for line in (read('/proc/stat') or '').splitlines():
        fields = line.split()
        if fields and fields[0].startswith('cpu'):
            # guest counters are already included in user/nice: exclude them.
            values = list(map(int, fields[1:9]))
            result[fields[0]] = (sum(values), values[3], values[4])
    return result


def sample(previous):
    current = cpu_ticks()
    cpu = {}
    for key, (total, idle, iowait) in current.items():
        old = previous.get(key)
        dt = total-old[0] if old else 0
        cpu[key] = {
            'busy_pct': round(100*(dt-(idle-old[1])-(iowait-old[2]))/dt, 2) if dt > 0 else None,
            'iowait_pct': round(100*(iowait-old[2])/dt, 2) if dt > 0 else None}
    memory = {}
    for line in (read('/proc/meminfo') or '').splitlines():
        fields = line.split()
        if fields and fields[0].rstrip(':') in ('MemTotal','MemAvailable','SwapTotal','SwapFree'):
            memory[fields[0].rstrip(':')+'_kB'] = int(fields[1])
    sensors = {}
    for path in pathlib.Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        raw = read(path)
        try:
            sensors[str(path.parent.name)] = {'type': read(path.parent/'type'), 'celsius': float(raw)/1000}
        except (ValueError, TypeError):
            sensors[str(path.parent.name)] = {'celsius': None}
    frequencies = {str(p.parent.parent.name): read(p) for p in
                   pathlib.Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')}
    throttle = {'status': 'N/A', 'raw': None}
    if shutil.which('vcgencmd'):
        try:
            proc = subprocess.run(['vcgencmd','get_throttled'], capture_output=True, text=True, timeout=.5)
            throttle = {'status': 'OK' if proc.returncode == 0 else 'ERROR', 'raw': proc.stdout.strip(),
                        'stderr': proc.stderr.strip()}
            if proc.returncode == 0:
                bits = int(proc.stdout.strip().split('=')[1], 16)
                throttle['active_bits_0_to_3'] = bits & 15
                throttle['historical_bits_16_to_19'] = (bits >> 16) & 15
        except (OSError, subprocess.TimeoutExpired, ValueError, IndexError) as exc:
            throttle = {'status': 'ERROR', 'reason': str(exc)}
    return current, {'unix_s': time.time(), 'mono_ns': time.monotonic_ns(), 'cpu': cpu,
                     'memory': memory, 'thermal': sensors, 'frequency_kHz': frequencies,
                     'psi': {kind: read('/proc/pressure/'+kind) for kind in ('cpu','memory','io')},
                     'throttle': throttle}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', default='host_health.jsonl')
    args = parser.parse_args()
    if not 0 < args.seconds <= 86400:
        parser.error('seconds must be in (0,86400]')
    previous = cpu_ticks()
    deadline = time.monotonic()+args.seconds
    next_tick = time.monotonic()+1
    # Exclusive creation avoids replacing an earlier run.
    with open(args.output, 'x', buffering=1) as output:
        try:
            while time.monotonic() < deadline:
                time.sleep(max(0, min(next_tick, deadline)-time.monotonic()))
                previous, row = sample(previous)
                line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                output.write(line+'\n')
                print(line, flush=True)
                next_tick += 1
                if next_tick < time.monotonic():
                    next_tick = time.monotonic()+1
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()

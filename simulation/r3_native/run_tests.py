#!/usr/bin/env python3
"""Compile and execute the real firmware lift code with deterministic IO stubs.
Production TU is the unmodified source prefix containing the complete lift logic;
IMU tasks are excluded. This is a logic test, not an ESP32 timing/step-loss test.
"""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[2]
tests=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='tai_r3_native_') as temp:
    build=Path(temp)
    production=(root/'esp32_firmware/Projects/bno055_imu_monitor/src/main.cpp').read_text()
    (build/'production_lift_under_test.hpp').write_text(production.split('void updateLiftSnapshot()')[0])
    for name in ('standalone','production'):
        exe=build/name
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-I',str(tests),'-I',str(build),str(tests/f'test_{name}.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

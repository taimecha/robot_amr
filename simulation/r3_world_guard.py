#!/usr/bin/env python3
"""Refuse a duplicate Gazebo in this partition; never terminate existing processes."""
import os
from pathlib import Path


def existing_servers(partition):
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            args=proc.joinpath('cmdline').read_bytes().split(b'\0')
            executable=Path(os.fsdecode(args[0])).name
            is_gz=executable.startswith('gz sim') or (executable in ('gz','ruby') and b'sim' in args)
            if not is_gz:continue
            environment=proc.joinpath('environ').read_bytes().split(b'\0')
            if ('GZ_PARTITION='+partition).encode() in environment:found.append(int(proc.name))
        except (OSError,IndexError):continue
    return found

if __name__=='__main__':
    if existing_servers('tai_r3_sim_only_89'):
        raise SystemExit('Đã có Gazebo trong partition r3. Đóng world cũ trước khi mở ca mới; không tự đóng tiến trình của bạn.')

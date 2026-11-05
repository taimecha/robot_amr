#!/usr/bin/env python3
"""Reproduce the offline portions of the new docking plan on this laptop."""
import csv
import hashlib
import json
import platform
import subprocess
from pathlib import Path

from docking_preview import run as run_preview
from quality_replay import main as run_quality
from plan_sim import main as run_plan
from render_gazebo_trial import main as render_gazebo
from short_motion_matrix import main as run_short_motion
from integrated_scenario import main as run_integrated


ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'simulation/output'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    run_preview(OUT)
    run_quality(OUT)
    run_plan(OUT)
    run_short_motion(OUT)
    integrated=run_integrated(OUT)
    config=[ROOT/'camera/detect_astra/config/pallet_alignment.yaml',
            ROOT/'camera/detect_astra/config/pallet_alignment_collision.yaml',
            ROOT/'ros2_ws/config/ros2_controllers.yaml',
            ROOT/'ros2_ws/description/forklift.xacro']
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    manifest={'run_type':'OFFLINE_SYNTHETIC_LAPTOP','host':platform.node(),
              'architecture':platform.machine(),'commit':commit,
              'working_tree_clean':not bool(subprocess.check_output(
                  ['git','status','--porcelain'],cwd=ROOT,text=True).strip()),
              'config_sha256':{str(p.relative_to(ROOT)):digest(p) for p in config},
              'pi5_runtime_verified':False,'robot_hardware_publishers':0,
              'gazebo_trial_present':(OUT/'gazebo_trial.csv').exists()}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    summary=json.loads((OUT/'summary.json').read_text())
    quality=json.loads((OUT/'quality_report.json').read_text())
    plan=json.loads((OUT/'plan_report.json').read_text())
    with (OUT/'short_motion_matrix.csv').open() as f:
        short_rows=list(csv.DictReader(f))
    short_skip=sum(r['result']=='SKIP_CURVATURE' for r in short_rows)
    gazebo_note='Gazebo pre-dock chưa chạy trong bộ dữ liệu này.'
    gazebo_file=OUT/'gazebo_trial.csv'
    if gazebo_file.exists():
        with gazebo_file.open() as f:
            measured=list(csv.DictReader(f))
        if measured:
            import math
            last=measured[-1]
            error=math.hypot(.9-float(last['odom_x_m']),.16-float(last['odom_y_m']))
            max_v=max(abs(float(r['cmd_v_mps'])) for r in measured)
            max_w=max(abs(float(r['cmd_w_rps'])) for r in measured)
            closest=min(float(r['closest_scan_m']) for r in measured if r['closest_scan_m'])
            gazebo_note=(f"Gazebo pre-dock: {len(measured)} mẫu trong {last['time_s']} s, "
                         f"dừng ở `{last['state']}`, sai số XY {error*1000:.1f} mm; "
                         f"max |v|={max_v:.3f} m/s, |ω|={max_w:.3f} rad/s, "
                         f"scan gần nhất {closest:.2f} m. Sau lượt, odom velocity về gần 0.")
            render_gazebo(OUT)
    with (OUT/'events.jsonl').open('w') as f:
        with (OUT/'timeline.csv').open() as stream:
            for row in csv.DictReader(stream):
                f.write(json.dumps({'kind':'acquisition',**row})+'\n')
        for case in summary['fault_cases']:
            f.write(json.dumps({'kind':'shadow_fault',**case})+'\n')
    with (OUT/'metrics.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(('metric','value','unit','provenance'))
        for metric,value,unit in (
            ('acquisition_ready',summary['acquisition_ready_s'],'s'),
            ('shadow_duration',summary['shadow_duration_s'],'s'),
            ('shadow_final_lateral_error',summary['shadow_final_lateral_error_m'],'m'),
            ('quality_absolute_x_error',quality['ab_variants']['gate_depth_pose']['x_absolute_error_m'],'m'),
            ('quality_absolute_x_budget_pass',quality['absolute_x_budget_pass'],'bool'),
            ('plan_feasible',plan['illustrative_feasible'],'cases'),
            ('plan_reposition',plan['reposition_required'],'cases')):
            writer.writerow((metric,value,unit,'synthetic simulation'))
    report=f"""# Kết quả mô phỏng docking trên laptop

- Commit: `{commit[:7]}`; chạy trên `{platform.node()}` ({platform.machine()}); Pi 5 runtime: **chưa kiểm tra**.
- Nhánh acquisition danh định **giả định hiệu chuẩn độc lập đạt**: giữ `ACQUIRING` sau 5 s, READY ở {summary['acquisition_ready_s']} s sau khi tắt YOLO và nhận RGB-D mới; cần thao tác bắt đầu riêng.
- Hai mode được kiểm tra riêng: DIAGNOSTIC giữ đo pallet khi chuyển động nhưng không dùng phép đo đó để lái; FROZEN_TARGET tắt đo/tracking pallet trước khi bám đường cố định.
- Kịch bản nối các bước: dữ liệu có bias {integrated['biased_uncorrected']['absolute_x_error_m']*1000:.1f} mm bị chặn tại `{integrated['biased_uncorrected']['state']}`; dữ liệu giả đã sửa bias còn {integrated['synthetic_corrected']['absolute_x_error_m']*1000:.1f} mm đi qua planner và shadow follower, dừng tại `{integrated['synthetic_corrected']['state']}`. Không phát lệnh nâng.
- Replay frame: {quality['reason_counts']['accepted']} mẫu được nhận. Lý do loại: {quality['reason_counts']}. Sai số X tuyệt đối sau lọc {quality['ab_variants']['gate_depth_pose']['x_absolute_error_m']*1000:.1f} mm so với truth giả, vượt budget 5 mm. **Nhánh dữ liệu này bị chặn ở ACQUIRING**; dữ liệu ổn định không sửa được bias 10 mm.
- Planner quintic + straight: {plan['illustrative_feasible']}/{plan['cases']} trường hợp khả thi **trong mô hình 2D giả**; {plan['reposition_required']} trường hợp REPOSITION_REQUIRED. Clearance danh định lỗ/càng {plan['geometry']['nominal_lateral_clearance_m']*1000:.1f} mm mỗi bên, budget giả {plan['assumed_uncertainty_total_m']*1000:.1f} mm.
- Ma trận chuyển động ngắn 30/50/100 cm với hướng 0, ±5/10/15°: {len(short_rows)} trường hợp offline, {short_skip} bị SKIP vì độ cong vượt giới hạn giả định; có ghi độ lệch ngang cuối.
- Shadow follower: {summary['shadow_duration_s']} s, dừng ở `{summary['shadow_final_state']}`. Không phát lệnh robot, không xác nhận insertion và không nâng. Sáu ca lỗi đều latch STOP.
- {gazebo_note} Lượt Gazebo dùng đích giả đã đặt sẵn và chỉ đi đến pre-dock; chưa nối nhận diện camera với planner.

Các số trên không phải phép đo camera, động cơ hoặc Pi thật. Đường đi chưa được xác nhận theo chiều cao càng, vật cản chưa quan sát, slip, độ võng tải, độ chính xác hiệu chuẩn hoặc ground truth thật. Cấu hình collision monitor hiện chỉ nhận `/scan_filtered`; chưa bảo vệ được vùng mù pallet. Vì vậy không dùng kết quả này để bật chế độ chạy thật.
"""
    (OUT/'report.md').write_text(report)
    print('Wrote',OUT/'report.md')


if __name__=='__main__':
    main()

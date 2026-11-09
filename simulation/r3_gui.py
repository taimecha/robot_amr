#!/usr/bin/env python3
"""Desktop simulation console: camera recognition, unrestricted case inputs, two paths."""
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from r3_case import ForkCase
from r3_plot import draw_two_paths

ROOT=Path(__file__).resolve().parents[1]
LAUNCH=ROOT/'simulation/launch_r3_sim.sh'
G=json.loads((ROOT/'simulation/r3/fixture.json').read_text())


def read_json(path,default=None):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return default


class Console:
    def __init__(self,root):
        self.root=root;root.title('R3 · Mô phỏng docking và nhận diện pallet');root.geometry('1380x900')
        self.folder=ROOT/'simulation/output/r3_user'/datetime.now().strftime('gui_%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(parents=True)
        self.reset_pending=False;self.vision_restart_pending=False
        self.jobs={};self.scenario=None;self.case_file=None;self.vision_dir=None;self.plan_dir=None;self.run_dir=None
        self.planned=[];self.actual=[];self.target=None;self.plan_source=None;self.pending_run=False;self.closed=False;self.image_stamp=None;self.capture_stamp=None;self.truth_offset=0;self.truth_header=None
        top=ttk.Frame(root,padding=10);top.pack(fill='x')
        ttk.Label(top,text='MÔ PHỎNG · Tâm gốc càng so với tâm lỗ vào pallet',font=('',15,'bold')).pack(anchor='w')
        fields=ttk.Frame(top);fields.pack(fill='x',pady=8)
        self.inputs=[]
        for col,(label,value) in enumerate([('Cách mặt pallet (m)','1.5'),('Lệch ngang (m), + trái','0'),('Góc lệch (°), + quay trái','0')]):
            ttk.Label(fields,text=label).grid(row=0,column=col*2,sticky='w',padx=(0,5))
            var=tk.StringVar(value=value);self.inputs.append(var)
            ttk.Entry(fields,textvariable=var,width=13).grid(row=0,column=col*2+1,padx=(0,20))
        self.show_gz=tk.BooleanVar(value=False)
        ttk.Checkbutton(fields,text='Mở cửa sổ Gazebo 3D',variable=self.show_gz).grid(row=0,column=6)
        ttk.Label(top,text='Khoảng cách đo theo pháp tuyến mặt pallet; 0° = càng hướng vào pallet, mặt phẳng gốc càng song song mặt pallet. Nhập số hữu hạn, không giới hạn min/max.',foreground='#555').pack(anchor='w')
        actions=ttk.Frame(top);actions.pack(fill='x',pady=(10,0))
        for label,command in [('1. Áp dụng / chạy lại vị trí',self.open_world),('Đo lại camera',self.restart_vision),('2. Tạo quỹ đạo',self.prepare),('3. Chạy bám mô phỏng',self.start),('DỪNG MÔ PHỎNG',self.stop_motion)]:
            ttk.Button(actions,text=label,command=command).pack(side='left',padx=(0,7))
        self.applied_pose=tk.StringVar(value='Chưa áp dụng tư thế. Đổi ca ngay tại đây; không cần reset terminal.')
        ttk.Label(top,textvariable=self.applied_pose,foreground='#235782').pack(anchor='w',pady=(5,0))
        self.source=tk.StringVar(value='Pose nhập')
        ttk.Combobox(actions,textvariable=self.source,values=['Pose nhập','RGB-D đã khóa'],state='readonly',width=16).pack(side='left')
        ttk.Label(top,text='Pose nhập là dữ liệu kịch bản. RGB-D đã khóa là kết quả đo từ ảnh Depth + TF; hai nguồn được ghi riêng.',foreground='#555').pack(anchor='w',pady=(6,0))
        panes=ttk.Panedwindow(root,orient='horizontal');panes.pack(fill='both',expand=True,padx=10)
        left=ttk.Frame(panes);right=ttk.Frame(panes);panes.add(left,weight=1);panes.add(right,weight=2)
        ttk.Label(left,text='CAMERA MÔ PHỎNG / NHẬN DIỆN',font=('',12,'bold')).pack(anchor='w',pady=5)
        tabs=ttk.Notebook(left);tabs.pack(fill='both',expand=True)
        live=ttk.Frame(tabs);capture=ttk.Frame(tabs);tabs.add(live,text='Camera trực tiếp');tabs.add(capture,text='Ảnh nhận diện cuối')
        self.camera=ttk.Label(live,text='Áp dụng tư thế để mở camera mô phỏng',anchor='center');self.camera.pack(fill='both',expand=True)
        self.capture=ttk.Label(capture,text='Chưa có lần nhận diện',anchor='center');self.capture.pack(fill='both',expand=True)
        self.detection=tk.StringVar(value='Chưa có ảnh và kết quả nhận diện.')
        ttk.Label(left,textvariable=self.detection,justify='left',wraplength=440).pack(fill='x',pady=10)
        self.figure=Figure(figsize=(8,5),dpi=100);self.ax=self.figure.subplots();self.figure.subplots_adjust(bottom=.13,left=.10,right=.97,top=.90)
        self.canvas=FigureCanvasTkAgg(self.figure,master=right);self.canvas.get_tk_widget().pack(fill='both',expand=True)
        draw_two_paths(self.ax,[],[],None,G)
        self.result=tk.StringVar(value='Chưa chạy. Vận tốc tối đa 0,15 m/s; sai góc ≤ 0,5°; mục tiêu gap gốc càng 20 mm.')
        ttk.Label(right,textvariable=self.result,wraplength=750,justify='left').pack(fill='x',pady=8)
        self.status=tk.StringVar(value='Nhập tư thế → mở world → xem nhận diện / tạo quỹ đạo → chạy bám mô phỏng.')
        ttk.Label(root,textvariable=self.status,wraplength=1300,padding=10).pack(fill='x')
        ttk.Label(root,text=f'Kết quả: {self.folder}',foreground='#555',padding=(10,0,10,8)).pack(anchor='w')
        root.protocol('WM_DELETE_WINDOW',self.close);root.after(400,self.poll)

    def spawn(self,key,args,logdir):
        logdir.mkdir(parents=True,exist_ok=True)
        with (logdir/(key+'.log')).open('a') as log:
            self.jobs[key]=subprocess.Popen([str(a) for a in args],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return self.jobs[key]

    def busy(self,key):return key in self.jobs and self.jobs[key].poll() is None

    def terminate(self,key):
        p=self.jobs.get(key)
        if p and p.poll() is None:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
            self.root.after(3500,lambda:self.escalate(p))

    @staticmethod
    def escalate(p):
        # Clean only the process group created by this console.
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass

    def checked_case(self):
        c=ForkCase(*(float(v.get().replace(',','.')) for v in self.inputs)).scenario(G)
        if self.scenario is None or c['case_id']!=self.scenario['case_id']:
            raise ValueError('Tư thế nhập đã thay đổi. Bấm “Áp dụng tư thế / mở world” trước.')
        return c

    def open_world(self):
        try:
            if self.reset_pending:
                self.status.set('Đang đổi ca trong cùng cửa sổ; chờ world cũ đóng xong...');return
            if self.busy('run') or self.busy('prepare') or self.pending_run:raise ValueError('Dừng ca chạy / chờ tạo quỹ đạo xong trước khi đổi world.')
            case=ForkCase(*(float(v.get().replace(',','.')) for v in self.inputs)).scenario(G)
            if self.busy('world') or self.busy('vision'):
                self.reset_pending=True;self.vision_restart_pending=False
                self.terminate('vision');self.terminate('world')
                self.status.set('Đang đóng world của ca cũ; sẽ áp dụng vị trí mới trong cùng giao diện...')
                self.root.after(100,self.wait_world_closed);return
            case['headless']=not self.show_gz.get();self.scenario=case
            self.case_dir=self.folder/datetime.now().strftime('case_%H%M%S_%f');self.case_dir.mkdir()
            self.case_file=self.case_dir/'case.json';self.case_file.write_text(json.dumps(case,indent=2))
            self.plan_dir=None;self.plan_source=None;self.run_dir=None;self.planned=[];self.actual=[];self.target=case['target_odom']
            self.truth_offset=0;self.truth_header=None;self.vision_dir=None;self.image_stamp=None;self.capture_stamp=None
            self.camera.configure(image='',text='Đang mở camera cho ca mới');self.capture.configure(image='',text='Chưa nhận diện trong ca mới')
            self.detection.set('Ca mới: chờ ảnh camera và nhận diện. Vị trí RGB-D chưa có.')
            v=case['input']
            self.applied_pose.set(f"Tư thế đã áp dụng (dữ liệu mô phỏng): cách {v['distance_m']:g} m · ngang {v['lateral_m']:+g} m · góc {v['yaw_deg']:+g}°. Không phải phép đo camera.")
            self.result.set('Ca mới chưa có quỹ đạo. Bấm “Tạo quỹ đạo” rồi xem trước khi chạy.')
            self.redraw()
            args=[LAUNCH,'world','--case',self.case_file]
            if case['headless']:args+=['--headless']
            self.spawn('world',args,self.case_dir)
            self.status.set('Đang mở world và camera mô phỏng. Đổi vị trí lần sau bằng chính nút Áp dụng / chạy lại vị trí.')
            token=self.case_file
            self.root.after(2500,lambda:self.start_current_camera(token))
        except (ValueError,OSError) as e:self.status.set(str(e))

    def wait_world_closed(self):
        if self.closed:return
        if self.busy('world') or self.busy('vision'):
            self.root.after(100,self.wait_world_closed);return
        # poll() must not treat the previous world exit as a failure of the new one.
        self.jobs.pop('world',None);self.jobs.pop('vision',None)
        self.reset_pending=False;self.open_world()

    def start_current_camera(self,token):
        if not self.closed and not self.reset_pending and self.case_file==token and not self.busy('vision') and not self.vision_restart_pending:
            self.restart_vision()

    def wait_camera_closed(self,token):
        if self.closed or self.case_file!=token or self.reset_pending:return
        if self.busy('vision'):
            self.root.after(100,lambda:self.wait_camera_closed(token));return
        self.vision_restart_pending=False;self.restart_vision()

    def restart_vision(self):
        try:
            self.checked_case()
            if self.reset_pending or self.vision_restart_pending:raise ValueError('Đang đổi ca / khởi động lại camera, chờ hoàn tất.')
            if self.busy('run') or self.pending_run:raise ValueError('Chỉ đo lại camera khi ca bám đã dừng.')
            if not self.busy('world'):raise ValueError('World chưa mở hoặc không thuộc giao diện này. Xem world.log; nếu đang mở world khác, hãy đóng world đó trước.')
            if self.busy('vision'):
                self.vision_restart_pending=True;self.terminate('vision')
                token=self.case_file;self.root.after(100,lambda:self.wait_camera_closed(token));return
            self.vision_dir=self.case_dir/datetime.now().strftime('camera_%H%M%S_%f');self.vision_dir.mkdir()
            python=Path('/home/tai/astra_pallets/.venv/bin/python')
            if not python.exists():raise ValueError('Thiếu môi trường YOLO: /home/tai/astra_pallets/.venv/bin/python')
            self.spawn('vision',[python,ROOT/'simulation/r3_vision.py','--case',self.case_file,'--output',self.vision_dir],self.vision_dir)
            self.image_stamp=None;self.capture_stamp=None;self.status.set('Camera đang nhận diện. Kết quả đo sẽ hiện ở khung bên trái.')
        except (ValueError,OSError) as e:self.status.set(str(e))

    def prepare(self):
        try:
            self.checked_case()
            if self.busy('run') or self.busy('prepare') or self.pending_run:raise ValueError('Chờ ca hiện tại kết thúc hoặc bấm Dừng.')
            if self.reset_pending:raise ValueError('Đang đổi ca, chờ world mới mở xong.')
            self.plan_source=self.source.get();args=[sys.executable,ROOT/'simulation/r3_prepare.py','--case',self.case_file]
            if self.plan_source=='RGB-D đã khóa':
                vision=read_json(self.vision_dir/'vision.json',{}) if self.vision_dir else {}
                if not vision.get('locked_target'):raise ValueError('Chưa khóa được pose RGB-D. Xem trạng thái nhận diện hoặc chọn Pose nhập.')
                # Copy the displayed capture; the prepared plan can never follow later measurements.
                capture=self.case_dir/datetime.now().strftime('capture_%H%M%S_%f.json');capture.write_text(json.dumps(vision,indent=2));args+=['--vision',capture]
            self.plan_dir=self.case_dir/datetime.now().strftime('plan_%H%M%S_%f')
            self.spawn('prepare',args+['--output',self.plan_dir],self.plan_dir)
            self.planned=[];self.actual=[];self.run_dir=None;self.redraw();self.result.set('Đang tạo quỹ đạo cho ca hiện tại; chưa chạy.');self.status.set('Đang tạo Smac State Lattice → Constrained Smoother đoạn tiếp cận → đoạn thẳng cuối...')
        except (ValueError,OSError) as e:self.status.set(str(e))

    def start(self):
        try:
            self.checked_case()
            if self.reset_pending:raise ValueError('Đang đổi ca, chờ world mới mở xong.')
            if self.busy('run') or self.pending_run:raise ValueError('Ca bám đang chạy.')
            if not self.busy('world'):raise ValueError('World của giao diện chưa mở. Mở world trước.')
            if self.run_dir:raise ValueError('Xe đã chạy trong world này. Áp dụng lại tư thế để bắt đầu ca mới.')
            if self.plan_source!=self.source.get():raise ValueError('Nguồn pose đã thay đổi. Tạo lại quỹ đạo để xem trước.')
            ready=read_json(self.plan_dir/'prepare_result.json',{}) if self.plan_dir else {}
            if self.busy('prepare') or ready.get('status')!='PREVIEW_WAIT':raise ValueError('Cần tạo và xem trước một quỹ đạo hợp lệ.')
            if self.busy('vision'):
                (self.vision_dir/'vision_control.json').write_text(json.dumps({'measure':False}))
            self.pending_run=True;self.status.set('Đang tắt đo pallet trước khi chạy bám...')
        except (ValueError,OSError) as e:self.status.set(str(e))

    def begin_run(self):
        if self.closed or self.reset_pending:
            self.pending_run=False;return
        try:self.checked_case()
        except ValueError as e:
            self.pending_run=False;self.status.set(str(e));return
        self.pending_run=False;self.run_dir=self.case_dir/datetime.now().strftime('run_%H%M%S_%f');self.actual=[];self.truth_offset=0;self.truth_header=None
        self.spawn('run',[LAUNCH,'run','--case',self.case_file,'--plan',self.plan_dir/'frozen_plan.json','--output',self.run_dir,'--no-open'],self.run_dir)
        self.status.set('Đang bám trong Gazebo. Hai đường cùng tham chiếu tâm gốc càng; không đặt thời hạn chạy cố định.')

    def stop_motion(self):
        self.pending_run=False;self.terminate('run');self.status.set('Đã yêu cầu dừng ca mô phỏng.')

    def redraw(self):
        draw_two_paths(self.ax,self.planned,self.actual,self.target,G,scene=self.scenario);self.canvas.draw_idle()

    def update_camera(self):
        if not self.vision_dir:return
        v=read_json(self.vision_dir/'vision.json',{})
        states={'ACQUIRE_YOLO':'Đang tìm pallet bằng YOLO','STABILIZE_RGBD':'Đang ổn định vị trí RGB-D','LOCKED':'Đã khóa vị trí RGB-D','MEASUREMENT_OFF':'Đã dừng đo pallet','LOADING_MODEL':'Đang nạp model','ERROR':'Lỗi camera / model'}
        lines=[f"Trạng thái: {states.get(v.get('state'),'Chờ camera')}",v.get('message','')]
        if v.get('label'):lines.append(f"Nhận diện: {v['label']} · tin cậy {v['confidence']:.1%}")
        elif v.get('detections'):
            lines.append('Phát hiện: '+', '.join(f"{d['label']} {d['confidence']:.1%}" for d in v['detections']))
        elif 'detections' in v:lines.append('YOLO chưa nhận ra pallet trong ảnh (0 phát hiện).')
        m=v.get('measurement')
        if m:lines+=['KẾT QUẢ ĐO RGB-D (tâm gốc càng):',f"Cách mặt: {m['distance_m']:.4f} m | lệch ngang: {m['lateral_m']*1000:+.1f} mm",f"Góc lệch: {m['yaw_deg']:+.3f}° | {m['depth_points']} điểm Depth"]
        if not m:
            lines.append('Khoảng cách / lệch ngang / góc đo: —')
            lines.append('Cần YOLO xác nhận mục tiêu rồi mới đo vị trí RGB-D.')
        if v.get('capture_stamp_s') is not None:lines.append(f"Mốc chụp: {v['capture_stamp_s']:.3f} s mô phỏng")
        if not v.get('pallet_measurement_enabled',True) and m:lines.append('Kết quả giữ từ lần đo đứng yên; camera vẫn truyền ảnh.')
        self.detection.set('\n'.join(lines))
        image_path=self.vision_dir/'camera.jpg'
        try:
            stamp=image_path.stat().st_mtime_ns
            if stamp!=self.image_stamp:
                im=Image.open(image_path);im.load();im.thumbnail((max(320,self.camera.winfo_width()),max(220,self.camera.winfo_height())))
                self.photo=ImageTk.PhotoImage(im);self.camera.configure(image=self.photo,text='');self.image_stamp=stamp
        except OSError:pass
        try:
            path=self.vision_dir/'detection.jpg';stamp=path.stat().st_mtime_ns
            if stamp!=self.capture_stamp:
                im=Image.open(path);im.load();im.thumbnail((max(320,self.camera.winfo_width()),max(220,self.camera.winfo_height())))
                self.capture_photo=ImageTk.PhotoImage(im);self.capture.configure(image=self.capture_photo,text='');self.capture_stamp=stamp
        except OSError:pass
        if self.pending_run and (not self.busy('vision') or (not v.get('yolo_enabled',True) and not v.get('pallet_measurement_enabled',True))):self.begin_run()

    def update_truth(self):
        if not self.run_dir:return
        path=self.run_dir/'gazebo_truth.csv'
        try:
            with path.open() as f:
                f.seek(self.truth_offset)
                if self.truth_header is None:
                    header=f.readline()
                    if not header.endswith('\n'):return
                    self.truth_header=next(csv.reader([header]))
                changed=False
                while True:
                    pos=f.tell();line=f.readline()
                    if not line or not line.endswith('\n'):f.seek(pos);break
                    r=dict(zip(self.truth_header,next(csv.reader([line]))))
                    self.actual.append([float(r['fork_x_m']),float(r['fork_y_m']),float(r['fork_yaw_rad'])]);changed=True
                self.truth_offset=f.tell()
            if changed:self.redraw()
        except (OSError,KeyError,ValueError):pass
        result=read_json(self.run_dir/'mission_result.json')
        if result:
            truth=result.get('independent_scorer',{});stop=truth.get('stopped_truth')
            text=f"Kết quả: {result.get('status','')} · {result.get('reason','')}"
            if stop:
                text+=f"\nDừng: lệch ngang {stop['ey_m']*1000:+.2f} mm; góc {math.degrees(stop['yaw_error_rad']):+.3f}°; gap {stop['gap_m']*1000:.2f} mm."
                text+="\nSai số dọc chỉ báo cáo theo yêu cầu; tiêu chí ngang, góc và tốc độ vẫn giữ."
            self.result.set(text)

    def poll(self):
        if self.closed:return
        self.update_camera();self.update_truth()
        if self.pending_run and not self.busy('vision'):self.begin_run()
        p=self.jobs.get('prepare')
        if p and p.poll() is not None:
            del self.jobs['prepare'];r=read_json(self.plan_dir/'prepare_result.json',{})
            if r.get('status')=='PREVIEW_WAIT':
                with (self.plan_dir/'path_planned_fork_root.csv').open() as f:self.planned=[[float(row[k]) for k in ('x_m','y_m','yaw_rad')] for row in csv.DictReader(f)]
                self.target=read_json(self.plan_dir/'target_snapshot.json')['pose'];self.redraw()
                if self.busy('vision'):(self.vision_dir/'vision_control.json').write_text(json.dumps({'measure':False}))
                self.result.set(f"Nguồn: {self.plan_source}. Dài {r['length_m']:.3f} m; {r['rotations']} pha xoay. Chưa chạy.")
                self.status.set('Đã tạo đường màu xanh. Xem quỹ đạo rồi bấm Chạy bám mô phỏng.')
            else:self.status.set('Không có quỹ đạo hợp lệ: '+r.get('reason',f'Xem {self.plan_dir}/prepare.log'))
        for key in ('world','run'):
            p=self.jobs.get(key)
            if p and p.poll() is not None:
                del self.jobs[key]
                if key=='world':
                    self.terminate('vision')
                    if not self.reset_pending:self.status.set('World đã đóng / không mở được. Xem world.log. Nếu có world cũ, đóng world đó trước.')
                else:
                    if (self.run_dir/'mission_result.json').exists():self.status.set('Ca bám kết thúc. Kết quả và hình hai đường nằm trong '+str(self.run_dir))
                    else:self.status.set('Ca bám chưa có kết quả; xem lỗi tại '+str(self.run_dir/'run.log'))
        self.root.after(400,self.poll)

    def close(self):
        self.closed=True;self.pending_run=False
        for key in list(self.jobs):self.terminate(key)
        self.status.set('Đang đóng các tiến trình mô phỏng do giao diện mở...')
        self.root.after(4000,self.root.destroy)


def main():
    root=tk.Tk();Console(root);root.mainloop()
if __name__=='__main__':main()

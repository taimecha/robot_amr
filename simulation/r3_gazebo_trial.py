#!/usr/bin/env python3
"""Execute a frozen r3 plan ONLY inside the isolated Gazebo fixture domain.
No ground truth is subscribed by this controller. Requires explicit CLI action.
"""
import argparse
import json
import math
import os
import shutil
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from r3_core import FrozenPlan, digest, errors, fork_for_base
from r3_sim import FixedPlanFollower, write_csv, plot

class Trial(Node):
    def __init__(self, plan, out, tracking_only=False, longitudinal_stop_report_only=False):
        super().__init__('r3_gazebo_fixed_plan_trial',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.plan=plan;self.g=json.loads(plan.geometry_json)
        self.tracking_only=tracking_only
        self.longitudinal_stop_report_only=longitudinal_stop_report_only
        self.phases=json.loads(plan.phases_json);self.snapshot=json.loads(plan.target_json);self.target=self.snapshot['pose']
        self.follow=FixedPlanFollower(self.phases,self.target,self.g)
        self.out=out;self.rows=[];self.odom=None;self.odom_wall=0.;self.lift=0.;self.lift_wall=0.;self.scan_wall=0.
        self.live_stream=(out/'live_odom.jsonl').open('w',buffering=1)
        self.blocked=False;self.finished=False;self.start_time=None;self.stop_time=None;self.start_wall=time.monotonic()
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.on_joint,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_filtered',self.on_scan,qos_profile_sensor_data)
        # Publishers are created only after the process guard below has verified
        # isolated domain/partition AND the active controller's simulated clock.
        self.cmd=self.create_publisher(Twist,'/cmd_vel',1)
        self.lift_cmd=None if tracking_only else self.create_publisher(Float64MultiArray,'/lift_controller/commands',1)
        self.state='WAIT_SENSORS';self.reason='';self.last_time=None
        self.create_timer(.05,self.tick)

    def on_odom(self,msg):
        self.odom=msg;self.odom_wall=time.monotonic()

    def on_joint(self,msg):
        if 'lift_joint' in msg.name:
            self.lift=msg.position[msg.name.index('lift_joint')];self.lift_wall=time.monotonic()

    def on_scan(self,msg):
        self.scan_wall=time.monotonic()
        # Preserve the old demo's LiDAR stop condition, no stale-policy change.
        self.blocked=any(math.isfinite(d) and msg.range_min<d<.60 for i,d in enumerate(msg.ranges)
                         if abs(msg.angle_min+i*msg.angle_increment)<math.radians(28))

    def send(self,v=0.,w=0.):
        m=Twist();m.linear.x=v;m.angular.z=w;self.cmd.publish(m)

    def fail(self,reason):
        self.state='STOP_LATCHED';self.reason=reason;self.finished=True;self.send()

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        wall=time.monotonic()
        if self.finished:self.send();return
        if self.last_time is not None and now < self.last_time:
            self.fail('simulation clock moved backwards; mixed worlds/epoch');return
        if not self.odom or wall-self.odom_wall>.2 or wall-self.scan_wall>.45 or wall-self.lift_wall>.5:
            self.send()
            if self.start_time is not None:self.fail('stale odom/scan/lift')
            return
        q=self.odom.pose.pose.orientation;p=self.odom.pose.pose.position
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=(p.x,p.y,yaw)
        if self.start_time is None:
            start=self.phases[0]['poses'][0]
            if math.dist(start[:2],pose[:2])>.02 or abs(yaw-start[2])>math.radians(.5):
                self.fail('odom start does not match frozen plan');return
            self.start_time=now;self.follow.start(True)
        dt=.05 if self.last_time is None else max(.001,min(.1,now-self.last_time));self.last_time=now
        e=errors(pose,self.target,self.g)
        v=w=0.
        if self.state=='LIFTING':
            self.send()
            if self.blocked:self.fail('obstacle during lift');return
            m=Float64MultiArray();m.data=[.080];self.lift_cmd.publish(m)
            if now-self.stop_time>20.:self.fail('lift feedback timeout');return
            if abs(self.lift-.080)<.001:
                self.state='LIFT_REACHED_PALLET_RISE_UNVERIFIED';self.finished=True
        else:
            v,w=self.follow.step(pose,dt,obstacle=self.blocked)
            self.state=self.follow.state
            if self.state=='STOP_LATCHED':
                self.fail(self.follow.reason)
            elif self.state=='STOPPED':
                if self.stop_time is None:self.stop_time=now
                self.send()
                stopped=abs(self.odom.twist.twist.linear.x)<.001 and abs(self.odom.twist.twist.angular.z)<.001
                if stopped and now-self.stop_time>=.5:
                    gap_bad=not self.longitudinal_stop_report_only and abs(e['gap_m']-.020)>.005
                    if gap_bad or abs(e['ey_m'])>.020 or abs(e['yaw_error_rad'])>math.radians(.5):
                        self.fail('after-braking fork goal criteria failed')
                    elif self.tracking_only:
                        self.state='DONE_TRACKING';self.finished=True
                    else:self.state='LIFTING'
            else:self.send(v,w)
        f=fork_for_base(pose,self.g)
        self.rows.append(dict(time_s=now-self.start_time,sim_stamp_s=now,state=self.state,x_m=p.x,y_m=p.y,yaw_rad=yaw,
                             fork_x_m=f[0],fork_y_m=f[1],fork_yaw_rad=f[2],cmd_v_mps=v,cmd_w_rps=w,lift_m=self.lift,**e))
        self.live_stream.write(json.dumps(self.rows[-1])+'\n')


def verify_sim_controller():
    # This node has NO command publishers. Refuse hardware or unknown clocks.
    node=Node('r3_verify_simulation_clock')
    try:
        client=node.create_client(GetParameters,'/controller_manager/get_parameters')
        if not client.wait_for_service(timeout_sec=8.):raise RuntimeError('no isolated controller_manager')
        request=GetParameters.Request();request.names=['use_sim_time']
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        result=future.result()
        if result is None or not result.values or not result.values[0].bool_value:
            raise RuntimeError('controller_manager must use simulated time')
        until=time.monotonic()+15.
        topics=('/clock','/odom','/joint_states','/base_controller/odom')
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.1)
            counts={topic:len(node.get_publishers_info_by_topic(topic)) for topic in topics}
            if any(v>1 for v in counts.values()):raise RuntimeError(f'duplicate simulation publishers: {counts}')
            if all(v==1 for v in counts.values()):return
        raise RuntimeError(f'simulation publishers not ready: {counts}')
    finally:node.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--gazebo-execute',action='store_true')
    parser.add_argument('--tracking-only',action='store_true',help='stop after path following; no mission lift publisher')
    parser.add_argument('--longitudinal-stop-report-only',action='store_true',help='user-approved waiver: keep20mm nominal goal, report longitudinal error without rejecting it')
    args=parser.parse_args()
    if not args.gazebo_execute or os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('GZ_PARTITION')!='tai_r3_sim_only_89' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        parser.error('requires --gazebo-execute, domain89, partition tai_r3_sim_only_89, discovery LOCALHOST')
    value=json.loads(args.plan.read_text());plan=FrozenPlan(**{k:value[k] for k in ('target_json','geometry_json','phases_json','config_json')})
    if plan.plan_id!=value['plan_id']:parser.error('plan hash mismatch')
    if json.loads(plan.geometry_json)['provenance']!='SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION':parser.error('synthetic fixture required')
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:
        verify_sim_controller();node=Trial(plan,args.output,args.tracking_only,args.longitudinal_stop_report_only)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if node:node.fail('operator stopped simulation')
    finally:
        if node:
            for name in ('case.json','frozen_plan.json','target_snapshot.json','path_planned_base.csv','path_planned_fork_root.csv'):
                source=args.plan.parent/name
                if source.exists():shutil.copyfile(source,args.output/name)
            for _ in range(5):node.send();rclpy.spin_once(node,timeout_sec=.05)
            if node.rows:
                write_csv(args.output/'executed_odom_base.csv',node.rows)
                write_csv(args.output/'executed_odom_fork_root.csv',[dict(time_s=r['time_s'],x_m=r['fork_x_m'],y_m=r['fork_y_m'],yaw_rad=r['fork_yaw_rad']) for r in node.rows])
            result=dict(status='INCOMPLETE',provenance='Gazebo odom/IMU feedback; synthetic fixture',
                        target_source=node.snapshot.get('source','UNKNOWN'),
                        tracking_only=args.tracking_only,longitudinal_stop_report_only=args.longitudinal_stop_report_only,
                        state=node.state,reason=node.reason,plan_id=plan.plan_id,
                        final=node.rows[-1] if node.rows else None,
                        missing=['real extrinsics/tines confirmation','independent truth/contact/pallet-rise scorer'],real_robot_commands=0)
            (args.output/'mission_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
            if node.rows:
                try:plot(args.output,node.phases,node.rows,node.target,node.g,{},odom_label='Gazebo /odom')
                except (Exception,KeyboardInterrupt) as e:
                    (args.output/'plot_export_error.txt').write_text(type(e).__name__+': '+str(e))
            node.destroy_node()
            node.live_stream.close()
        rclpy.shutdown()
if __name__=='__main__':main()

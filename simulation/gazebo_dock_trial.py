#!/usr/bin/env python3
"""Isolated Gazebo pre-dock, with optional straight fork insertion and pallet lift."""
import argparse
import csv
from datetime import datetime
import json
import math
import os
import shutil
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String, Float64MultiArray

from docking_preview import ShadowFollower, candidate


def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class GazeboPreDock(Node):
    def __init__(self, output, execute, one_pallet=False, insert_and_lift=False):
        super().__init__('gazebo_pre_dock_trial')
        self.execute=execute
        self.output=output
        self.one_pallet=one_pallet
        self.insert_and_lift=insert_and_lift
        self.reason=''
        self.pallet_z=None;self.pallet_world_xy=None;self.pallet_wall=None;self.pallet_initial_z=None
        self.case=None
        self.lift_position=None;self.lift_wall=None
        self.stage_started=None;self.lift_target=.085
        self.case_parameters=None
        self.archive=None
        # A 16 cm lateral correction towards one synthetic pallet column. The
        # robot stops at x=0.9 m: fork tip remains ~0.14 m before face x=1.795.
        if one_pallet:
            self.path=None
            self.target_locked=False
            self.target_pose=None
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool,'/pallet_demo/target_locked',self.on_lock,latched)
            self.create_subscription(RosPath,'/pallet_demo/planned_path',self.on_path,latched)
            self.create_subscription(PoseStamped,'/pallet_demo/frozen_target',self.on_target,latched)
            self.create_subscription(String,'/pallet_demo/case',self.on_case,latched)
            if insert_and_lift:
                self.create_subscription(PoseStamped,'/pallet/pose',self.on_pallet_pose,10)
                self.create_subscription(JointState,'/joint_states',self.on_joint,qos_profile_sensor_data)
                self.lift_cmd=self.create_publisher(Float64MultiArray,'/lift_controller/commands',10)
        else:
            path=candidate(-.16,0.,.9)['rows']
            self.path=[(x,y+.16,yaw,kappa) for x,y,yaw,kappa,_ in path]
        self.follower=ShadowFollower(self.path) if self.path is not None else None
        self.cmd=self.create_publisher(Twist,'/cmd_vel',1) if execute else None
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_filtered',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.on_clock,10)
        self.odom=None;self.odom_wall=None;self.scan_wall=None;self.clock_wall=None
        self.last_odom_stamp=None;self.closest_obstacle=None
        self.rows=[];self.started_wall=None;self.finished_wall=None
        self.stale_since=None
        self.state='WAIT_SENSORS'
        self.create_timer(.05,self.tick)

    def on_lock(self,msg):
        if msg.data:self.target_locked=True

    def on_target(self,msg):
        if self.state=='WAIT_SENSORS':self.target_pose=msg

    def on_case(self,msg):
        if self.state=='WAIT_SENSORS':
            try:self.case_parameters=json.loads(msg.data)
            except (ValueError,TypeError):self.case_parameters=None

    def on_path(self,msg):
        if self.state!='WAIT_SENSORS' or len(msg.poses)<2:return
        self.path=[(p.pose.position.x,p.pose.position.y,
                    yaw_from_quaternion(p.pose.orientation),0.) for p in msg.poses]
        self.follower=ShadowFollower(self.path)

    def on_clock(self,_msg):
        self.clock_wall=time.monotonic()

    def on_pallet_pose(self,msg):
        # PosePublisher also emits the pallet link relative to the model.
        # Only the model pose in the Gazebo world measures lift off the floor.
        if msg.header.frame_id!='warehouse':return
        self.pallet_z=msg.pose.position.z
        self.pallet_world_xy=(msg.pose.position.x,msg.pose.position.y)
        self.pallet_wall=time.monotonic()

    def on_joint(self,msg):
        if 'lift_joint' in msg.name:
            i=msg.name.index('lift_joint')
            if i<len(msg.position):
                self.lift_position=msg.position[i]
                self.lift_wall=time.monotonic()

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last_odom_stamp is not None and stamp<self.last_odom_stamp:
            if self.follower is not None:self.follower.stop('odom clock reset')
            self.state='STOP_LATCHED'
        self.last_odom_stamp=stamp
        self.odom=msg;self.odom_wall=time.monotonic()

    def on_scan(self,msg):
        self.scan_wall=time.monotonic()
        self.closest_obstacle=min((r for r in msg.ranges if math.isfinite(r) and r>0),default=None)

    def zero(self):
        if self.cmd is not None:self.cmd.publish(Twist())

    def fail(self,reason):
        self.reason=reason
        if self.follower is not None:self.follower.stop(reason)
        self.state='STOP_LATCHED';self.zero()

    def record(self,wall,p,yaw,v=0.,w=0.):
        gap=self.fork_root_gap(p) if self.case is not None and self.pallet_world_xy is not None else None
        self.rows.append((round(wall-self.started_wall,3),self.state,p.x,p.y,yaw,
                          v,w,self.closest_obstacle,self.lift_position,self.pallet_z,gap))

    def pallet_in_odom(self):
        dx=self.pallet_world_xy[0]-self.case.spawn_x
        dy=self.pallet_world_xy[1]-self.case.spawn_y
        c,s=math.cos(self.case.yaw_rad),math.sin(self.case.yaw_rad)
        return c*dx+s*dy,-s*dx+c*dy

    def fork_root_gap(self,p):
        from one_pallet_case import PALLET_LENGTH,FORK_ROOT_FROM_BASE
        heading=self.path[-1][2]
        tx,ty=self.pallet_in_odom()
        along=(tx-p.x)*math.cos(heading)+(ty-p.y)*math.sin(heading)
        return along-PALLET_LENGTH/2-FORK_ROOT_FROM_BASE

    def insertion_geometry(self,p,yaw):
        from one_pallet_case import FORK_ROOT_CLEARANCE
        heading=self.path[-1][2]
        tx,ty=self.pallet_in_odom()
        side=-(tx-p.x)*math.sin(heading)+(ty-p.y)*math.cos(heading)
        remaining=self.fork_root_gap(p)-FORK_ROOT_CLEARANCE
        angle=math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))
        return remaining,side,angle

    def tick(self):
        wall=time.monotonic()
        if self.state in ('DONE','DONE_LIFTED','STOP_LATCHED'):
            self.zero()
            if self.finished_wall is None:self.finished_wall=wall
            return
        if (self.clock_wall is None or wall-self.clock_wall>.5
                or self.odom_wall is None or wall-self.odom_wall>.2
                or self.scan_wall is None or wall-self.scan_wall>.45):
            if self.started_wall is not None:
                if self.stale_since is None:self.stale_since=wall
                if wall-self.stale_since>1.:
                    self.fail('stale clock/odom/scan for over 1 s')
            self.zero();return
        self.stale_since=None
        if self.insert_and_lift and self.started_wall is not None and (
                self.pallet_wall is None or wall-self.pallet_wall>.5 or
                self.lift_wall is None or wall-self.lift_wall>.5):
            self.fail('stale pallet pose or lift joint');return
        if self.state=='WAIT_SENSORS':
            if not self.execute:
                self.zero();return
            if self.path is None or self.follower is None or (hasattr(self,'target_locked') and
                    (not self.target_locked or self.target_pose is None)):
                self.zero();return
            if self.insert_and_lift and (self.pallet_wall is None or self.lift_wall is None or
                    wall-self.pallet_wall>.5 or wall-self.lift_wall>.5):
                self.zero();return
            p=self.odom.pose.pose.position
            if math.hypot(p.x-self.path[0][0],p.y-self.path[0][1])>.05:
                self.follower.stop('robot is not at path start')
                self.state='STOP_LATCHED';self.zero();return
            if hasattr(self,'target_locked'):
                from one_pallet_case import PALLET_LENGTH,FORK_TIP_FROM_BASE
                x,y,heading,_=self.path[-1]
                target=self.target_pose.pose.position
                if self.insert_and_lift:
                    if self.case_parameters is None:
                        self.fail('missing simulation case for insertion check');return
                    from one_pallet_case import OnePalletCase
                    case=OnePalletCase(self.case_parameters['distance_m'],
                                       self.case_parameters['lateral_m'],
                                       self.case_parameters['yaw_deg'])
                    self.case=case
                    expected=case.target_in_odom()
                    if math.hypot(target.x-expected[0],target.y-expected[1])>.03:
                        self.fail('frozen RGB-D target more than 3 cm from Gazebo case');return
                forward=(target.x-x)*math.cos(heading)+(target.y-y)*math.sin(heading)
                side=-(target.x-x)*math.sin(heading)+(target.y-y)*math.cos(heading)
                gap=forward-PALLET_LENGTH/2-FORK_TIP_FROM_BASE
                if gap<.10 or abs(side)>.03:
                    self.follower.stop('pre-dock path too close to pallet face')
                    self.state='STOP_LATCHED';self.zero();return
            self.follower.start(True)
            self.started_wall=wall
            self.state='FOLLOW_CURVE'
        p=self.odom.pose.pose.position
        yaw=yaw_from_quaternion(self.odom.pose.pose.orientation)
        if self.closest_obstacle is not None and self.closest_obstacle<.60:
            self.fail('scan obstacle within 0.60 m');return
        if self.state in ('INSERTING','LIFTING'):
            remaining,side,angle=self.insertion_geometry(p,yaw)
            if self.state=='INSERTING':
                if abs(side)>.018 or abs(angle)>math.radians(2.):
                    self.fail('insertion alignment drift');return
                if remaining<-.004 or wall-self.stage_started>22.:
                    self.fail('insertion overshoot or timeout');return
                if remaining<=.004:
                    self.zero();self.state='LIFTING';self.stage_started=wall
                    self.pallet_initial_z=self.pallet_z
                    self.get_logger().info(f'Forks inserted; root gap={self.fork_root_gap(p):.3f} m; lifting from pallet z={self.pallet_initial_z:.3f} m')
                    self.record(wall,p,yaw);return
                msg=Twist();msg.linear.x=min(.04,max(.012,remaining*.5));self.cmd.publish(msg)
                self.record(wall,p,yaw,msg.linear.x,0.);return
            self.zero()
            lift=Float64MultiArray();lift.data=[self.lift_target];self.lift_cmd.publish(lift)
            if wall-self.stage_started>18.:
                self.fail('pallet did not rise 5 cm within lift timeout');return
            rise=self.pallet_z-self.pallet_initial_z
            if abs(side)>.02 or abs(angle)>math.radians(2.5):
                self.fail('robot moved during lift');return
            if .045<=rise<=.065 and self.lift_position>=self.lift_target-.008:
                self.state='DONE_LIFTED';self.finished_wall=wall
                self.reason=f'pallet lifted {rise:.3f} m after straight fork insertion'
                self.get_logger().info(f'Pallet lifted {rise:.3f} m; lift joint {self.lift_position:.3f} m')
            self.record(wall,p,yaw);return
        if wall-self.started_wall>35:
            self.fail('pre-dock motion time limit');return
        v,w=self.follower.step(p.x,p.y,yaw)
        # A simulated actuator command. The guard in main rejects non-Gazebo domains.
        msg=Twist();msg.linear.x=v;msg.angular.z=w
        self.cmd.publish(msg)
        self.state=self.follower.state
        self.record(wall,p,yaw,v,w)
        if self.state=='WAIT_INSERTION_VERIFICATION':
            self.zero()
            if not self.insert_and_lift:
                self.state='DONE';self.finished_wall=wall
            else:
                remaining,side,angle=self.insertion_geometry(p,yaw)
                if not .47<=remaining<=.63 or abs(side)>.018 or abs(angle)>math.radians(2.):
                    self.fail(f'pre-insertion geometry: remaining={remaining:.3f} side={side:.3f} angle={math.degrees(angle):.1f}deg')
                elif self.pallet_z is None or abs(self.pallet_z)>.015 or self.lift_position is None or abs(self.lift_position)>.01:
                    self.fail('pallet or forks not at floor starting height')
                else:
                    self.state='INSERTING';self.stage_started=wall
                    self.get_logger().info(f'Straight insertion: {remaining:.3f} m remaining, side={side:.3f} m')

    def save(self):
        self.output.parent.mkdir(parents=True,exist_ok=True)
        with self.output.open('w',newline='') as f:
            writer=csv.writer(f)
            writer.writerow(('time_s','state','odom_x_m','odom_y_m','odom_yaw_rad',
                             'cmd_v_mps','cmd_w_rps','closest_scan_m','lift_joint_m','pallet_z_m','fork_root_gap_m'))
            writer.writerows(self.rows)
        if self.one_pallet and self.rows and self.case_parameters:
            c=self.case_parameters
            name=(f'd{c["distance_m"]:.2f}_y{c["lateral_m"]:+.2f}_'
                  f'yaw{c["yaw_deg"]:+.1f}_{datetime.now():%Y%m%d_%H%M%S}.csv')
            archive_dir=self.output.parent/'one_pallet_trials'
            archive_dir.mkdir(parents=True,exist_ok=True)
            self.archive=archive_dir/name
            shutil.copyfile(self.output,self.archive)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-execute',action='store_true')
    parser.add_argument('--one-pallet',action='store_true',
                        help='Follow the same pre-dock path shown in the one-pallet RViz demo')
    parser.add_argument('--insert-and-lift',action='store_true',
                        help='After pre-dock, insert forks straight and lift the Gazebo pallet 5 cm')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.insert_and_lift and not (args.one_pallet and args.gazebo_execute):
        parser.error('--insert-and-lift requires --one-pallet --gazebo-execute')
    if args.output is None:
        args.output=Path('simulation/output/one_pallet_trial.csv' if args.one_pallet and args.gazebo_execute
                         else 'simulation/output/gazebo_trial.csv' if args.gazebo_execute
                         else 'simulation/output/gazebo_trial_shadow.csv')
    if not args.gazebo_execute:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(('x_m','y_m','yaw_rad','kappa_1pm'))
            if args.one_pallet:
                from one_pallet_geometry import planned_rows
                writer.writerows(planned_rows())
            else:
                writer.writerows((x,y+.16,yaw,kappa)
                                 for x,y,yaw,kappa,_ in candidate(-.16,0.,.9)['rows'])
        print('shadow path only; no ROS node or publisher:',args.output)
        return
    partition=('tai_one_pallet_88' if args.one_pallet else 'tai_docking_preview_88')
    if args.gazebo_execute and (os.environ.get('ROS_DOMAIN_ID')!='88'
                                or os.environ.get('GZ_PARTITION')!=partition):
        parser.error(f'Gazebo execution requires isolated ROS_DOMAIN_ID=88 and GZ_PARTITION={partition}')
    rclpy.init()
    node=GazeboPreDock(args.output,args.gazebo_execute,args.one_pallet,args.insert_and_lift)
    deadline=None if args.one_pallet else time.monotonic()+45
    try:
        while rclpy.ok() and (deadline is None or time.monotonic()<deadline):
            rclpy.spin_once(node,timeout_sec=.1)
            if node.finished_wall is not None and time.monotonic()-node.finished_wall>1.:
                break
    finally:
        for _ in range(10):node.zero();time.sleep(.05)
        node.save()
        print('state:',node.state,'reason:',node.reason or (node.follower.reason if node.follower else 'no frozen target/path'),
              'rows:',len(node.rows),'output:',args.output,'archive:',node.archive)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()

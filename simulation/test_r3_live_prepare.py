"""The provisional real path uses camera data and cannot admit an entered pose."""
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from xml.etree import ElementTree

from r3_core import (FrozenPlan, errors, goal_base, real_geometry_mode, rectangle,
                     validate_axis_lateral_residual)
from r3_live_prepare import (build_plan, provisional_geometry, reacquire_needed, rgbd_qualified,
                             snapshot_from_samples)
from r3_real_obstacle_filter import read_real_target
from r3_real_preflight import inspect_plan


class LivePrepareTest(unittest.TestCase):
    def test_empirical_axis_shift_corrects_right_bias_once_at_any_heading(self):
        geometry=provisional_geometry()
        for yaw in (-20.,0.,5.64,20.):
            samples=[dict(stamp=100.+i*.08, odom=[.3,-.4,math.radians(37.)],
                          forward=1.8,lateral=.246,raw=.221,
                          heading=math.radians(yaw),heading_source='floor_referenced_plane',
                          heading_depth_x_slope=.00008) for i in range(5)]
            old,start=snapshot_from_samples(samples,{'session':1})
            new,_=snapshot_from_samples(samples,{'session':1},lateral_residual_m=.025)
            before=errors(start,old['pose'],geometry)
            after=errors(start,new['pose'],geometry)
            self.assertAlmostEqual(after['ey_m'],before['ey_m']-.025)
            self.assertAlmostEqual(after['gap_m'],before['gap_m'])
            self.assertAlmostEqual(after['yaw_error_rad'],before['yaw_error_rad'])
            self.assertAlmostEqual(new['measurement_report']['camera_mount_offset_m'],.025)
            self.assertAlmostEqual(new['measurement_report']['pallet_axis_lateral_residual_m'],.025)
            self.assertNotEqual(old['calibration_hash'],new['calibration_hash'])
            validate_axis_lateral_residual(new,start)
            # Applying another shift without its provenance must be rejected.
            twice=json.loads(json.dumps(new))
            twice['pose'][1]+=.025
            with self.assertRaisesRegex(ValueError,'exactly once'):
                validate_axis_lateral_residual(twice,start)

    def test_snapshot_records_projection_assumptions_and_hashes_calibration_not_session(self):
        samples = [dict(stamp=100.+i*.08, odom=[0.,0.,0.], forward=1.8,
                        lateral=.025, raw=0., heading=0.,
                        heading_source='floor_referenced_plane',
                        heading_depth_x_slope=.00008, position_depth_x_slope=.00008,
                        depth_correction_shared=True, camera_pitch_deg=20.,
                        depth_reference_u=370., forward_bias_m=.03843)
                   for i in range(5)]
        a,_=snapshot_from_samples(samples, {'session':1})
        b,_=snapshot_from_samples(samples, {'session':2})
        self.assertEqual(a['calibration_hash'],b['calibration_hash'])
        context=a['measurement_report']['calibration_assumptions']
        self.assertTrue(context['depth_correction_shared'])
        self.assertFalse(context['camera_mount_yaw_verified'])
        self.assertFalse(context['camera_roll_verified'])
        changed=[dict(s,camera_pitch_deg=21.) for s in samples]
        c,_=snapshot_from_samples(changed, {'session':1})
        self.assertNotEqual(a['calibration_hash'],c['calibration_hash'])

    def test_recognition_switch_waits_for_response_during_startup(self):
        from r3_perception_switch import request

        future = SimpleNamespace(done=lambda: True,
                                 result=lambda: SimpleNamespace(success=True, message='ready'))
        client = SimpleNamespace(wait_for_service=lambda timeout_sec: True,
                                 call_async=lambda _: future)
        node = SimpleNamespace(create_client=lambda *args: client)
        with patch('r3_perception_switch.rclpy.spin_until_future_complete') as spin:
            self.assertEqual(request(node, '/pallet/set_enabled', False, 120.), 'ready')
            spin.assert_called_once_with(node, future, timeout_sec=120.)

    def test_c_camera_verification_requires_fresh_agreement(self):
        from r3_real_trial import check_c_rgbd
        samples = [dict(received_wall=t, stamp_s=t, heading_rad=math.radians(.2),
                        lateral_m=.003, gap_m=.535) for t in (99.5, 99.7, 99.9)]
        report, reason = check_c_rgbd(samples, .535, math.radians(-.2), 100.)
        self.assertEqual(reason, '')
        self.assertAlmostEqual(report['fork_tip_clearance_cm'], 15.)
        self.assertAlmostEqual(report['camera_odom_yaw_disagreement_deg'], 0.)
        self.assertIsNone(check_c_rgbd(samples, .535, 0., 100.5)[0])
        skewed = [dict(sample, heading_rad=math.radians(2.)) for sample in samples]
        bad_report, bad_reason = check_c_rgbd(skewed, .535, 0., 100.)
        self.assertIn('insertion limits', bad_reason)
        self.assertAlmostEqual(bad_report['camera_heading_deg'], 2.)
        self.assertIn('disagreement', check_c_rgbd(samples, .535, math.radians(1.), 100.)[1])
        self.assertIsNotNone(check_c_rgbd(samples, .60, math.radians(-.2), 100.)[0])
        self.assertIn('gap difference', check_c_rgbd(samples, .60, math.radians(-.2), 100.)[1])

    def test_safe_command_graph_identifies_each_command_endpoint(self):
        from r3_real_trial import command_graph_status, trial_command_consumers

        def endpoint(name):
            return SimpleNamespace(node_namespace='/', node_name=name)

        class Probe:
            def get_publishers_info_by_topic(self, topic):
                return ([endpoint('r3_collision_monitor')] if topic == '/cmd_vel_safe'
                        else [endpoint('cmd_vel_stamper')])

            def get_subscriptions_info_by_topic(self, topic):
                return [endpoint('cmd_vel_stamper')]

        expected = command_graph_status(Probe())
        self.assertTrue(all(names == [owner] for _, _, owner, names in expected))

        class DuplicateProbe(Probe):
            def get_subscriptions_info_by_topic(self, topic):
                return [endpoint('cmd_vel_stamper'), endpoint('unexpected_consumer')]

        duplicate = command_graph_status(DuplicateProbe())
        self.assertEqual(duplicate[1][3], ['/cmd_vel_stamper', '/unexpected_consumer'])

        class LateDiscoveryProbe:
            def __init__(self, name):
                self.name = name

            def get_subscriptions_info_by_topic(self, topic):
                self.topic = topic
                return [SimpleNamespace(node_namespace='/', node_name=self.name,
                                        topic_type='geometry_msgs/msg/Twist')]

        unknown = LateDiscoveryProbe('_NODE_NAME_UNKNOWN_')
        expected_consumer = [('/r3_collision_monitor', 'geometry_msgs/msg/Twist')]
        self.assertNotEqual(trial_command_consumers(unknown), expected_consumer)
        discovered = LateDiscoveryProbe('r3_collision_monitor')
        self.assertEqual(trial_command_consumers(discovered), expected_consumer)
        self.assertEqual(discovered.topic, '/pallet/r3/cmd_vel')

    def test_raw_imu_heading_steers_and_disagreement_stops_trial(self):
        import rclpy
        from sensor_msgs.msg import Imu
        from r3_real_trial import RealTrial, relative_imu_heading

        self.assertAlmostEqual(relative_imu_heading(.1, math.radians(179),
                                                    math.radians(-179)), .1+math.radians(2))
        plan = FrozenPlan(json.dumps({'pose': [2., 0., 0.]}),
                          json.dumps(provisional_geometry()),
                          json.dumps([{'type': 'APPROACH',
                                       'poses': [[0., 0., 0.], [1., 0., 0.]]}]), '{}')
        started_ros = not rclpy.ok()
        if started_ros:
            rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                trial = RealTrial(plan, Path(tmp))
                try:
                    trial.pose = (0., 0., 0.)
                    trial.imu_anchor = (0., 0.)
                    imu = Imu()
                    imu.header.frame_id = 'imu_link'
                    imu.header.stamp = trial.get_clock().now().to_msg()
                    imu.orientation.z = math.sin(math.radians(1.5))
                    imu.orientation.w = math.cos(math.radians(1.5))
                    trial.on_imu(imu)
                    self.assertAlmostEqual(trial.imu_control_pose()[2], math.radians(3.))
                    trial.armed = trial.started = True
                    trial.odom_wall = trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.
                    trial.yolo_enabled = False
                    with patch('r3_real_trial.time.monotonic', return_value=100.), \
                            patch.object(trial, 'send'):
                        trial.tick()
                    self.assertEqual(trial.state, 'STOP_LATCHED')
                    self.assertIn('IMU/odom heading disagreement', trial.reason)
                finally:
                    trial.destroy_node()
        finally:
            if started_ros:
                rclpy.shutdown()

    def test_lost_post_yolo_track_restarts_recognition_only_before_plan(self):
        lost = dict(phase='CALIBRATION_REQUIRED', enabled=False, busy=False,
                    yolo_stopped_ros_s=10.,
                    reason='YOLO đã tắt; RGB-D mất dấu, cần nhận diện lại sau hiệu chuẩn')
        self.assertTrue(reacquire_needed(lost))
        self.assertFalse(reacquire_needed(dict(lost, busy=True)))
        self.assertFalse(reacquire_needed(dict(lost, phase='READY')))
        self.assertFalse(reacquire_needed(dict(lost,
            reason='YOLO đã tắt; chưa xác nhận sai số tuyệt đối để chạy xe')))

    def test_plan_uses_fresh_post_yolo_capture_without_five_second_health_window(self):
        status = dict(enabled=False, busy=False, detections=3,
                      yolo_stopped_ros_s=100.5,
                      rgbd_health=dict(valid_ratio=0., stable=False))
        samples = [dict(stamp=t) for t in (100.0, 100.1, 100.2, 100.4, 100.6)]
        self.assertTrue(rgbd_qualified(status, samples, 100.8))
        self.assertFalse(rgbd_qualified(status, samples, 101.1))
        self.assertFalse(rgbd_qualified(status, samples[:-1], 100.8))
        self.assertFalse(rgbd_qualified(dict(status, enabled=True), samples, 100.8))
        self.assertTrue(rgbd_qualified(dict(status, phase='CALIBRATION_REQUIRED',
            reason='YOLO đã tắt; RGB-D mất dấu, cần nhận diện lại sau hiệu chuẩn'),
            samples, 100.8))

    def test_capture_keeps_recent_preoff_samples_and_requires_valid_postoff_frame(self):
        import rclpy
        from rclpy.time import Time
        from detected_pallets_interfaces.msg import PalletTarget
        from r3_live_prepare import Capture

        started_ros = not rclpy.ok()
        if started_ros:
            rclpy.init()
        node = Capture()
        try:
            now = node.get_clock().now().nanoseconds*1e-9
            node.status = dict(session=1, detections=3, enabled=True, busy=False)
            node.status_wall = time.monotonic()
            node.odom = (0., 0., 0., 0., 0.)
            for index, offset in enumerate((-.23, -.17, -.11, -.05, 0.)):
                stamp = now+offset
                if index == 4:
                    node.status = dict(session=1, detections=3, enabled=False,
                                       busy=False, yolo_stopped_ros_s=now-.025)
                node.odom_stamp = stamp
                msg = PalletTarget()
                msg.header.stamp = Time(seconds=stamp).to_msg()
                msg.header.frame_id = 'camera_color_optical_frame'
                msg.valid = True
                msg.rgb_depth_delta_ms = 0.
                msg.left_depth_samples = msg.right_depth_samples = 60
                msg.opening_width_m = .25
                msg.leg_center_separation_m = .32
                msg.chassis_front_forward_m = 1.5
                msg.entry_heading_rad = math.radians(.3)
                msg.opening_lateral_m = .085
                msg.status = ('TRACKED_RGBD; lateral_camera_raw_m=+0.0600; '
                              'lateral_total_bias_m=+0.0250; lateral_fork_m=+0.0850; '
                              'heading_source=floor_referenced_plane; '
                              'heading_depth_x_slope=0.0001')
                node.on_target(msg)
            self.assertEqual(len(node.samples), 5)
            self.assertTrue(rgbd_qualified(node.status, list(node.samples),
                                           node.get_clock().now().nanoseconds*1e-9))
            snapshot, _ = snapshot_from_samples(list(node.samples), node.status)
            self.assertAlmostEqual(snapshot['measurement_report']['fork_lateral_corrected_m'], .085)
            msg.valid = False
            node.on_target(msg)
            self.assertFalse(node.samples)
        finally:
            node.destroy_node()
            if started_ros:
                rclpy.shutdown()

    def test_motion_requires_lidar_but_not_rgbd_scan(self):
        import rclpy
        import yaml
        from nav2_msgs.msg import CollisionMonitorState
        from r3_real_trial import RealTrial, LIDAR_TIMEOUT_S
        root = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((root/'camera/detect_astra/config/r3_collision_monitor.yaml').read_text())
        sources = config['r3_collision_monitor']['ros__parameters']['observation_sources']
        self.assertEqual(sources, ['lidar'])
        self.assertEqual(config['r3_collision_monitor']['ros__parameters']['source_timeout'],
                         LIDAR_TIMEOUT_S)
        plan = FrozenPlan(json.dumps({'pose': [2., 0., 0.]}),
                          json.dumps(provisional_geometry()),
                          json.dumps([{'type': 'APPROACH',
                                       'poses': [[0., 0., 0.], [1., 0., 0.]]}]), '{}')
        started_ros = not rclpy.ok()
        if started_ros:
            rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                trial = RealTrial(plan, Path(tmp))
                try:
                    self.assertEqual(set(trial.scan_wall), {'lidar'})
                    with patch('r3_real_trial.time.monotonic', return_value=100.):
                        trial.on_scan('lidar')
                    with patch('r3_real_trial.time.monotonic', return_value=100.1):
                        trial.on_scan('lidar')
                    self.assertEqual(trial.lidar_stable_since, 100.)
                    with patch('r3_real_trial.time.monotonic', return_value=101.):
                        trial.on_scan('lidar')
                    self.assertEqual(trial.lidar_stable_since, 101.)
                    state = CollisionMonitorState()
                    state.action_type = CollisionMonitorState.STOP
                    state.polygon_name = 'invalid source'
                    trial.on_collision_state(state)
                    self.assertFalse(trial.finished)
                    self.assertTrue(trial.monitor_stop)
                    state.action_type = CollisionMonitorState.DO_NOTHING
                    trial.on_collision_state(state)
                    self.assertFalse(trial.monitor_stop)
                    state.action_type = CollisionMonitorState.STOP
                    state.polygon_name = 'StopZone'
                    trial.on_collision_state(state)
                    self.assertTrue(trial.finished)
                finally:
                    trial.destroy_node()
        finally:
            if started_ros:
                rclpy.shutdown()

    def test_transient_odom_gap_holds_zero_before_latching(self):
        import rclpy
        from r3_real_trial import RealTrial

        plan = FrozenPlan(json.dumps({'pose': [2., 0., 0.]}),
                          json.dumps(provisional_geometry()),
                          json.dumps([{'type': 'APPROACH',
                                       'poses': [[0., 0., 0.], [1., 0., 0.]]}]), '{}')
        started_ros = not rclpy.ok()
        if started_ros:
            rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                trial = RealTrial(plan, Path(tmp))
                try:
                    trial.armed = trial.started = True
                    trial.pose = (0., 0., 0.)
                    trial.imu_yaw = 0.
                    trial.imu_anchor = (0., 0.)
                    trial.imu_wall = 100.3
                    trial.odom_wall = 100.
                    trial.yolo_enabled = False
                    trial.yolo_wall = 100.3
                    trial.scan_wall['lidar'] = 100.3
                    with patch('r3_real_trial.time.monotonic', return_value=100.31), \
                            patch.object(trial, 'send') as send:
                        trial.tick()
                        send.assert_called_once_with()
                    self.assertFalse(trial.finished)
                    self.assertEqual(trial.sensor_hold_since, 100.31)
                    trial.odom_wall = trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.5
                    with patch('r3_real_trial.time.monotonic', return_value=100.5):
                        trial.tick()
                    self.assertFalse(trial.finished)
                    self.assertIsNone(trial.sensor_hold_since)
                    trial.odom_wall = 100.5
                    trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.8
                    with patch('r3_real_trial.time.monotonic', return_value=100.8):
                        trial.tick()
                    self.assertFalse(trial.finished)
                    trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 101.9
                    with patch('r3_real_trial.time.monotonic', return_value=101.9):
                        trial.tick()
                    self.assertTrue(trial.finished)
                    self.assertIn('odom=', trial.reason)
                finally:
                    trial.destroy_node()
        finally:
            if started_ros:
                rclpy.shutdown()

    def test_real_diagnostic_brakes_before_c_without_second_camera(self):
        import rclpy
        from r3_real_trial import RealTrial

        geometry = provisional_geometry()
        target = (1.8, 0., 0.)
        c = goal_base(target, geometry, .535)
        b = goal_base(target, geometry)
        plan = FrozenPlan(json.dumps({'pose': target}), json.dumps(geometry),
                          json.dumps([{'type': 'APPROACH', 'poses': [[0., 0., 0.], c]},
                                      {'type': 'STRAIGHT', 'poses': [c, b]}]), '{}')
        started_ros = not rclpy.ok()
        if started_ros:
            rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                trial = RealTrial(plan, Path(tmp), stop_at_c=True)
                try:
                    trial.armed = trial.started = True
                    trial.follow.start(True)
                    trial.pose = (c[0]-.04, c[1], c[2])
                    trial.imu_yaw = c[2]+math.radians(1.)
                    trial.imu_anchor = (c[2], c[2])
                    trial.twist = (0., 0.)
                    trial.odom_wall = trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.
                    trial.yolo_enabled = False
                    trial.last_tick = trial.last_progress_wall = 99.95
                    with patch('r3_real_trial.time.monotonic', return_value=100.), \
                            patch.object(trial, 'send') as send:
                        trial.tick()
                        send.assert_called_once_with()
                    self.assertEqual(trial.state, 'C_BRAKING')
                    self.assertFalse(trial.finished)
                    self.assertFalse(trial.c_rgbd_samples)
                    trial.twist = (.01, 0.)
                    trial.odom_wall = trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.6
                    with patch('r3_real_trial.time.monotonic', return_value=100.6), \
                            patch.object(trial, 'send'):
                        trial.tick()
                    self.assertEqual(trial.state, 'C_BRAKING')
                    self.assertFalse(trial.finished)
                    trial.twist = (0., 0.)
                    trial.odom_wall = trial.imu_wall = trial.yolo_wall = trial.scan_wall['lidar'] = 100.8
                    with patch('r3_real_trial.time.monotonic', return_value=100.8), \
                            patch.object(trial, 'send'):
                        trial.tick()
                    self.assertEqual(trial.state, 'C_DIAGNOSTIC_DONE')
                    self.assertTrue(trial.finished)
                    self.assertIsNone(trial.c_verified)
                    self.assertEqual(trial.rows[-1]['heading_source'], 'odom_ekf')
                    self.assertAlmostEqual(trial.rows[-1]['control_yaw_rad'], c[2])
                    self.assertAlmostEqual(trial.rows[-1]['imu_control_yaw_rad'],
                                           c[2]+math.radians(1.))
                finally:
                    trial.destroy_node()
        finally:
            if started_ros:
                rclpy.shutdown()

    def test_laptop_view_reflects_all_geometry_without_changing_coordinates(self):
        from matplotlib.figure import Figure
        from r3_render_real_result import configure_view
        ax = Figure().subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        configure_view(ax, 'laptop')
        origin = ax.transData.transform((0, 0))
        positive = ax.transData.transform((.5, .1))
        self.assertGreater(positive[0], origin[0])
        self.assertLess(positive[1], origin[1])
        configure_view(ax, 'laptop')  # Repeated setup must not flip it back.
        self.assertTrue(ax.yaxis_inverted())
        configure_view(ax, 'ros')
        self.assertGreater(ax.transData.transform((.5, .1))[1], origin[1])

    def test_pallet_view_keeps_pallet_level_and_rotates_robot(self):
        from r3_render_real_result import pallet_view
        target = (1.8, .08, math.radians(7.))
        deck = [pallet_view(p, target) for p in rectangle(target, (0., .355, -.1775, .1775))]
        self.assertAlmostEqual(deck[0][1], deck[1][1])
        self.assertAlmostEqual(deck[1][0]-deck[0][0], .355)
        # A robot initially aimed along odom X now shows its real yaw error.
        a = pallet_view((0., 0.), target)
        ahead = pallet_view((.3, 0.), target)
        self.assertAlmostEqual(math.atan2(ahead[1]-a[1], ahead[0]-a[0]), -target[2])
        # A positive relative heading must point up on the standard top view;
        # a negative heading must point down, even with a rotated pallet.
        for relative_deg in (-4., 4.):
            heading = target[2]+math.radians(relative_deg)
            root = (1., .1)
            tip = (root[0]+.388*math.cos(heading), root[1]+.388*math.sin(heading))
            root_xy, tip_xy = pallet_view(root, target), pallet_view(tip, target)
            displayed_heading = math.atan2(tip_xy[1]-root_xy[1], tip_xy[0]-root_xy[0])
            self.assertAlmostEqual(math.degrees(displayed_heading), relative_deg)

    def test_camera_snapshot_path_and_explicit_gate(self):
        now = time.time()
        samples = [dict(stamp=now-1+i*.08, odom=[0., 0., 0.],
                        forward=1.800, lateral=.0885,
                        heading=math.radians(-.43), raw=.0635,
                        heading_source='floor_referenced_plane',
                        heading_depth_x_slope=.000061)
                   for i in range(8)]
        snapshot, start = snapshot_from_samples(samples, {'session': 17})
        with self.assertRaisesRegex(ValueError, 'floor-referenced'):
            snapshot_from_samples([{k: v for k, v in sample.items()
                                    if k not in ('heading_source', 'heading_depth_x_slope')}
                                   for sample in samples], {'session': 17})
        self.assertAlmostEqual(snapshot['measurement_report']['fork_lateral_corrected_m'], .0885)
        self.assertAlmostEqual(snapshot['measurement_report']['camera_lateral_raw_m']+.025, .0885)
        self.assertEqual(snapshot['measurement_report']['heading_source'], 'floor_referenced_plane')
        geometry = provisional_geometry()
        with self.assertRaisesRegex(ValueError, 'START_TOO_CLOSE_FOR_C'):
            build_plan(snapshot, goal_base(snapshot['pose'], geometry, .405),
                       geometry, Path('/tmp/r3_no_output_expected'))
        self.assertEqual(real_geometry_mode(geometry, True), 'provisional')
        with self.assertRaises(ValueError):
            real_geometry_mode(geometry)
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'plan'
            plan, _, rows = build_plan(snapshot, start, geometry, output)
            phases = json.loads(plan.phases_json)
            self.assertAlmostEqual(json.loads(plan.config_json)['approach_refinement']['straight_lead_in_m'], .12)
            self.assertTrue(all(abs(p[2]-snapshot['pose'][2]) < 1e-6
                                for p in phases[0]['poses'][-8:]))
            self.assertGreaterEqual(
                errors(phases[1]['poses'][0], snapshot['pose'], geometry)['gap_m']
                - geometry['fork_length'], .15-1e-6)
            ElementTree.parse(output/'trajectory.svg')
            self.assertLess(abs(rows[-1]['ey_m']), .009)
            self.assertEqual(inspect_plan(output/'frozen_plan.json', True)['status'],
                             'STATIC_PLAN_VALID')
            self.assertEqual(inspect_plan(output/'frozen_plan.json')['status'], 'BLOCKED')
            self.assertEqual(read_real_target(output/'frozen_plan.json', True)[0]['pose'],
                             snapshot['pose'])
            with self.assertRaises(ValueError):
                read_real_target(output/'frozen_plan.json')
            with patch('r3_real_obstacle_filter.time.time', return_value=now+40):
                with self.assertRaisesRegex(ValueError, 'stale'):
                    read_real_target(output/'frozen_plan.json', True)
                self.assertEqual(read_real_target(output/'frozen_plan.json', True,
                                                  max_age_s=None)[0]['pose'], snapshot['pose'])
            trial = Path(tmp)/'trial'
            trial.mkdir()
            (trial/'mission_result.json').write_text(json.dumps({
                'status': 'DONE_TRACKING', 'plan_id': plan.plan_id}))
            (trial/'executed_odom_base.csv').write_text(
                'x_m,y_m,yaw_rad\n' + ''.join(f'{row["x_m"]},{row["y_m"]},{row["yaw_rad"]}\n'
                                     for row in rows[::20]))
            from r3_render_real_result import render
            result = render(output/'frozen_plan.json', trial, Path(tmp)/'result.png')
            self.assertEqual(result['odom_samples'], len(rows[::20]))
            self.assertTrue(Path(result['image']).is_file())
            ElementTree.parse(result['svg'])
            self.assertIn('không đo lại pallet lúc dừng', Path(result['svg']).read_text())

    def test_entered_pose_stays_blocked(self):
        path = (Path(__file__).parent/'output/r3_camera_measured_preview/'
                'centered_20261111_145707/plan_refined/frozen_plan.json')
        if path.exists():
            self.assertEqual(inspect_plan(path, True)['status'], 'BLOCKED')


if __name__ == '__main__':
    unittest.main()

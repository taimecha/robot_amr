"""Deterministic session timing and proof of fresh geometry; no neural model."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_acquisition import AcquisitionSession


class AcquisitionTest(unittest.TestCase):
    def session(self):
        session = AcquisitionSession(); session.accuracy_validated = True; session.start(0., 100.)
        return session

    def test_accuracy_unverified_stops_yolo_but_blocks_ready(self):
        s=self.session(); s.accuracy_validated=False
        for t in (9.,9.4,9.8):self.detection(s,t)
        self.targets(s,10.1)
        self.assertTrue(s.tick(10.1,110.1))
        self.assertEqual(s.phase,'STOPPING_YOLO')
        s.yolo_stopped(); s.verified()
        self.assertEqual(s.phase,'CALIBRATION_REQUIRED')
        self.assertFalse(s.locked)

    def detection(self, s, t, confidence=.9, valid=True):
        s.detection(100+t,t,100+t,valid,confidence,(10.,10.,110.,90.),'pallets')

    def targets(self, s, end):
        for i in range(11):
            t=end-.5+i*.05
            s.target(100+t,t,100+t,True,2.,.01,math.radians(1))

    def test_three_seconds_minimum_then_verify_and_wait(self):
        s=self.session()
        for t in (1.5,2.,2.5):self.detection(s,t)
        self.targets(s,2.9)
        self.assertFalse(s.tick(2.9,102.9))
        self.targets(s,3.)
        self.assertTrue(s.tick(3.,103.))
        self.assertEqual(s.phase,'STOPPING_YOLO'); self.assertFalse(s.locked)
        s.yolo_stopped(); self.assertEqual(s.phase,'VERIFYING_RGBD')
        s.verified(); self.assertEqual(s.phase,'READY'); self.assertTrue(s.locked)
        self.assertFalse(s.tick(3.1,103.1))

    def test_success_late_in_window(self):
        s=self.session()
        for t in (3.5,4.,4.5):self.detection(s,t)
        self.assertFalse(s.tick(4.5,104.5))
        self.targets(s,4.9)
        self.assertTrue(s.tick(4.9,104.9)); self.assertFalse(s.locked)

    def test_no_overall_timeout_without_pallet_or_depth(self):
        for detections in (False,True):
            s=self.session()
            if detections:
                for t in (3.5,4.,4.5):self.detection(s,t)
            self.assertFalse(s.tick(50.,150.))
            self.assertEqual(s.phase,'ACQUIRING'); self.assertFalse(s.locked)
            self.targets(s,5.2)
            self.assertEqual(s.tick(5.2,105.2), detections)
            self.assertFalse(s.locked)

    def test_late_success_after_five_seconds(self):
        s=self.session()
        for t in (9.,9.4,9.8):self.detection(s,t)
        self.targets(s,10.1)
        self.assertTrue(s.tick(10.1,110.1))
        self.assertEqual(s.phase,'STOPPING_YOLO')
        self.assertFalse(s.locked)

    def test_low_confidence_and_duplicate_detections_do_not_confirm(self):
        s=self.session()
        for _ in range(10):self.detection(s,2.)
        self.assertEqual(len(s.detections),1)
        self.detection(s,2.5,confidence=.7)
        self.assertEqual(len(s.detections),0)
        self.targets(s,3.)
        self.assertFalse(s.tick(3.,103.))

    def test_old_future_duplicate_and_jumping_geometry_is_rejected(self):
        s=self.session()
        s.target(99.,1.,101.,True,2.,0.,0.)
        self.assertFalse(s.target_ok)
        for _ in range(10):s.target(101.,1.,101.,True,2.,0.,0.)
        self.assertEqual(len(s.targets),1)
        s.target(101.1,1.1,101.1,True,2.2,0.,0.)
        self.assertEqual(len(s.targets),1)
        s.target(105.,1.2,101.2,True,2.,0.,0.)
        self.assertFalse(s.measurement_ready(1.2,101.2))

    def test_temporary_depth_error_preserves_identity_but_lost_clears_it(self):
        s=self.session()
        for t in (1.5,2.,2.5):self.detection(s,t)
        self.targets(s,3.); s.tick(3.,103.); s.yolo_stopped(); s.verified()
        s.target(103.1,3.1,103.1,False)
        self.assertTrue(s.locked); self.assertFalse(s.measurement_ready(3.1,103.1))
        self.targets(s,3.8)
        self.assertTrue(s.measurement_ready(3.8,103.8))
        s.target(104.,4.,104.,False,lost=True)
        self.assertEqual(s.phase,'LOST'); self.assertFalse(s.locked)
        self.assertFalse(s.tick(4.,104.))

    def test_retry_resets_previous_semantic_and_geometry(self):
        s=self.session()
        self.targets(s,2.)
        self.detection(s,2.)
        s.start(10.,110.)
        self.assertEqual(s.session,2)
        self.assertFalse(s.locked); self.assertFalse(s.targets); self.assertFalse(s.detections)
        s.target(102.,10.,110.,True,2.,0.,0.)
        self.assertFalse(s.targets)

    def test_delayed_semantic_identity_does_not_relax_geometry_freshness(self):
        s=self.session()
        for t in (1.8,2.3,2.8):
            s.detection(100+t-.7,t,100+t,True,.9,(10.,10.,110.,90.),'pallets')
        self.assertEqual(len(s.detections),3)
        s.target(102.3,3.,103.,True,2.,0.,0.)
        self.assertFalse(s.measurement_ready(3.,103.))
        self.targets(s,3.)
        self.assertTrue(s.tick(3.,103.))
        self.assertEqual(s.phase,'STOPPING_YOLO')
        self.assertFalse(s.locked)

if __name__=='__main__':unittest.main()

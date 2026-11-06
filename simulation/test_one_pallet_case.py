"""Pose changes must preserve the pallet-relative pre-dock endpoint."""
import math
import unittest

from one_pallet_case import (OnePalletCase, PALLET_LENGTH, FORK_TIP_FROM_BASE,
                             PRE_DOCK_GAP)


class OnePalletCaseTest(unittest.TestCase):
    def test_positions_and_headings(self):
        for distance,lateral,yaw in ((1.8225,-.10,0), (2.2,.15,10),
                                     (2.2,-.15,-10), (2.6,0,15)):
            with self.subTest(distance=distance,lateral=lateral,yaw=yaw):
                case=OnePalletCase(distance,lateral,yaw)
                target=case.target_in_odom()
                rows=case.path_in_odom(*target)
                self.assertAlmostEqual(rows[0][0],0.)
                self.assertAlmostEqual(rows[0][1],0.)
                self.assertAlmostEqual(rows[0][2],0.)
                end_x,end_y,end_yaw,_=rows[-1]
                self.assertAlmostEqual(end_yaw,-math.radians(yaw))
                along=(target[0]-end_x)*math.cos(end_yaw)+(target[1]-end_y)*math.sin(end_yaw)
                across=-(target[0]-end_x)*math.sin(end_yaw)+(target[1]-end_y)*math.cos(end_yaw)
                self.assertAlmostEqual(across,0.,places=6)
                self.assertAlmostEqual(along-PALLET_LENGTH/2-FORK_TIP_FROM_BASE,
                                       PRE_DOCK_GAP,places=6)

    def test_rejects_unreachable_pose(self):
        with self.assertRaises(ValueError):
            OnePalletCase(1.8,-.15,-10).path_in_odom(
                *OnePalletCase(1.8,-.15,-10).target_in_odom())

    def test_measured_heading_drives_pre_dock_path(self):
        case=OnePalletCase(1.7,-.08,8.)
        measured=math.radians(-7.5)
        target=case.target_in_odom()
        rows=case.path_in_odom(*target,measured)
        self.assertAlmostEqual(rows[-1][2],measured)
        self.assertAlmostEqual(case.insertion_end_in_odom(*target,measured)[2],measured)


if __name__=='__main__':
    unittest.main()

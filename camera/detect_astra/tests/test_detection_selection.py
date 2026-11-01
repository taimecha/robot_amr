from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_detection_selection import unique_pallet_indices


class PalletSelectionTest(unittest.TestCase):
    names = {0:'Box', 1:'emty pallet', 2:'pallets'}

    def test_duplicate_classes_keep_strongest_without_box_suppression(self):
        indices = unique_pallet_indices([(10,10,110,90)]*3, [0,1,2], [.99,.77,.63], self.names)
        self.assertEqual(indices, [1])

    def test_separate_and_partial_overlap_remain_ambiguous(self):
        for box in [(120,10,220,90), (40,10,140,90)]:
            indices = unique_pallet_indices([(10,10,110,90), box], [1,2], [.9,.8], self.names)
            self.assertEqual(indices, [0,1])

    def test_invalid_boxes_cannot_become_seed(self):
        indices = unique_pallet_indices([(0,0,0,0), (0,0,float('nan'),90)], [1,2], [.9,.8], self.names)
        self.assertEqual(indices, [])


if __name__ == '__main__':
    unittest.main()

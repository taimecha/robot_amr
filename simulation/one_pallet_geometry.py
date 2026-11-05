"""Shared one-pallet SDF geometry and illustrative pre-dock path (metres)."""
from docking_preview import candidate

PALLET_X = 2.0
PALLET_Y = .10
PALLET_LENGTH = .355
PALLET_WIDTH = .355
FRONT_OPENING = .255
FORK_PAIR_WIDTH = .155
FORK_TIP_FROM_BASE = .7555
PRE_DOCK_GAP = .16
STOP_X = PALLET_X - PALLET_LENGTH/2 - FORK_TIP_FROM_BASE - PRE_DOCK_GAP


def planned_rows():
    """S curve from robot origin to the pallet centerline, then straight."""
    return [(x, y+PALLET_Y, yaw, kappa)
            for x,y,yaw,kappa,_ in candidate(-PALLET_Y, 0, STOP_X)['rows']]

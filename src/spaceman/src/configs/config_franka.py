"""
Configuration for franka URDF & XML model.
"""
import sys
from pathlib import Path
import genesis as gs

current_file_path = Path(__file__).resolve().parent
sys.path.insert(0, str(current_file_path))

to_posix = lambda p: p.as_posix()

_ROOT_PATH = current_file_path.parent.parent
_ASSET_PATH = _ROOT_PATH / 'src' / 'assets'

_FRANKA_PATHS = {
    'urdf': _ASSET_PATH / 'urdf' / 'panda_bullet' / 'panda.urdf',
    'xml': _ASSET_PATH / 'xml' / 'franka_emika_panda' / 'panda.xml',
}

FRANKA_CONFIG = {
    "initial_dofs": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04],
    "control": {
        "kp": [4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100],
        "kv": [450, 450, 350, 350, 200, 200, 200, 10, 10],
        "force_range_min": [-87, -87, -87, -87, -12, -12, -12, -100, -100],
        "force_range_max": [87, 87, 87, 87, 12, 12, 12, 100, 100],
    }
}

IK_PARAMS = {
    "smooth_factor": 0.3,
    "max_joint_change": 0.05,
}

FRANKA_PARAMS = {
    "name": "franka",
    "end_effector": "panda_hand",
    "base": "panda_link0",
    "config": FRANKA_CONFIG,
    "joints": (
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        "panda_finger_joint1",
        "panda_finger_joint2",
    ),
    "motor": 7,
    "finger": 9,
    "finger_open": [0.04, 0.04],
    "finger_close": [0.0, 0.0],
    "ik_params": IK_PARAMS,
}

def _make_franka_urdf():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_FRANKA_PATHS['urdf']),
            pos=(-0.3, 0.0, 0.0),
            euler=(0, 0, 0),
            merge_fixed_links=False,
            fixed=True,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

def _make_franka_mjcf():
    return {
        "morph": gs.morphs.MJCF(
            file=to_posix(_FRANKA_PATHS['xml']),
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
        "vis_mode": "collision",
    }

__all__ = [
    'FRANKA_PARAMS',
    '_make_franka_urdf',
    '_make_franka_mjcf',
]

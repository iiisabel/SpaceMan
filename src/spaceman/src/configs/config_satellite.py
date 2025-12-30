"""
Configuration for satellite URDF model.
"""
import sys
from pathlib import Path
import genesis as gs

current_file_path = Path(__file__).resolve().parent
sys.path.insert(0, str(current_file_path))

to_posix = lambda p: p.as_posix()

_ROOT_PATH = current_file_path.parent.parent
_ASSET_PATH = _ROOT_PATH / 'src' / 'assets'

_SATELLITE_PATHS = {
    'urdf': _ASSET_PATH / 'urdf' / 'satellite' / 'urdf' / 'satellite.urdf',
    'battery_urdf': _ASSET_PATH / 'urdf' / 'satellite_battery' / 'urdf' / 'satellite_battery.urdf',
}

SATELLITE_PARAMS = {
    "name": "satellite",
    "base": "base_link"
}

SATELLITE_PID = {
    "name": "franka_meerge",
    "P": [100000, 10000, 10000, 100000, 0, 0],
    "I": [0, 0, 0, 0, 0, 0],
    "D": [1000, 1000, 1000, 100, 0, 0],
    "setpoint": [1, 0, 0, -2, 0, 0],
    "dt": 0.01,
    "limits": [
        None,         # x
        None,        # y
        None,        # z
        None,        # roll
        None,        # pitch
        None         # yaw
    ]
}

def _make_satellite():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_SATELLITE_PATHS['urdf']),
            scale=5e-1,
            merge_fixed_links=False,
            fixed=False,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

def _make_satellite_part():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_SATELLITE_PATHS['battery_urdf']),
            pos=(-1.1, 0.3, 1.0),
            euler=(0.0, 90.0, 0.0),
            scale=5e-1,
            merge_fixed_links=False,
            fixed=False,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

__all__ = [
    'SATELLITE_PARAMS',
    'SATELLITE_PID',
    '_make_satellite',
    '_make_satellite_part',
]
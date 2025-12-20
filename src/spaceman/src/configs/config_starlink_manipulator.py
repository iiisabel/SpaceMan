
"""
Configuration for starlink_combine_qf_space_manipulator URDF model.
"""
from pathlib import Path
import genesis as gs

to_posix = lambda p: p.as_posix()

_current_file_path = Path(__file__).resolve().parent
_ROOT_PATH = _current_file_path.parent.parent
_ASSET_PATH = _ROOT_PATH / 'src' / 'assets'

_STARLINK_MANIPULATOR_URDF = (
    _ASSET_PATH / 'urdf' / 'starlink_combine_qf_space_manipulator' 
    / 'starlink_combine_qf_space_manipulator.urdf'
)

FRANKA_S_Q_CONFIG = {
    "initial_dofs": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04, 0.04, 0.04, 0.04],
    "control": {
        "kp": [4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100, 100, 100, 100],
        "kv": [450, 450, 350, 350, 200, 200, 200, 10, 10, 10, 10, 10],
        "force_range_min": [-87, -87, -87, -87, -12, -12, -12, -100, -100, -100, -100, -100],
        "force_range_max": [87, 87, 87, 87, 12, 12, 12, 100, 100, 100, 100, 100],
    }
}

def _make_starlink_manipulator():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_STARLINK_MANIPULATOR_URDF),
            pos=(-0.3, 0.0, 0.0),
            euler=(0.0, 0.0, 0.0),
            merge_fixed_links=False,
            fixed=False,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

__all__ = [
    'FRANKA_S_Q_CONFIG',
    '_make_starlink_manipulator',
]
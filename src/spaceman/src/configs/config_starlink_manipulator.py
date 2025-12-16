# This python file contains parameters only for starlink_combine_qf_space_manipulator.urdf

from pathlib import Path
import genesis as gs

current_file_path = Path(__file__).resolve().parent
root_path = current_file_path.parent.parent
asset_path = root_path / 'src' / 'assets'
to_posix = lambda p: p.as_posix()

FRANKA_S_Q_CONFIG = {
    "initial_dofs": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "control": {
        "kp": [4500, 4500, 3500, 3500, 2000, 2000],
        "kv": [450, 450, 350, 350, 200, 200],
        "force_range_min": [-87, -87, -87, -87, -12, -12],
        "force_range_max": [87, 87, 87, 87, 12, 12],
    }
}

_urdf = asset_path / 'urdf' / 'starlink_combine_qf_space_manipulator' / 'starlink_combine_qf_space_manipulator.urdf'
def _make_starlink_manipulator():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_urdf),
            pos=(-0.3, 0.0, 0.0),
            euler=(0.0, 0.0, 0.0),
            merge_fixed_links=False,
            fixed=True,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

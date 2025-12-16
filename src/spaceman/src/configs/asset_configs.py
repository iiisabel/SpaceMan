from pathlib import Path
import genesis as gs

current_file_path = Path(__file__).resolve().parent
root_path = current_file_path.parent.parent
asset_path = root_path / 'src' / 'assets'

# path -> forward-slash string for URDF loaders
to_posix = lambda p: p.as_posix()

# lazy factory; constructs after gs.init()
def _make_franka_urdf():
    return {
        "morph": gs.morphs.URDF(
            file="urdf/panda_bullet/panda.urdf",
            pos=(-0.3, 0.0, 0.0),
            euler=(0, 0, 0),
            merge_fixed_links=False,
            fixed=True,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
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

def _make_franka_mjcf():
    return {
        "morph": gs.morphs.MJCF(
            file="xml/franka_emika_panda/panda.xml",
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
        "vis_mode": "collision",
    }

_satellite_urdf = asset_path / 'urdf' / 'satellite' / 'urdf' / 'satellite.urdf'
_satellite_battery_urdf = asset_path / 'urdf' / 'satellite_battery' / 'urdf' / 'satellite_battery.urdf'

def _make_satellite():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_satellite_urdf),
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
            file=to_posix(_satellite_battery_urdf),
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

_merge_urdf = asset_path / 'urdf' / 'starlink_combine_qf_space_manipulator' / 'starlink_combine_qf_space_manipulator.urdf'
def _make_franka_merge():
    return {
        "morph": gs.morphs.URDF(
            file=to_posix(_merge_urdf),
            pos=(-0.3, 0.0, 0.0),
            euler=(0.0, 0.0, 0.0),
            merge_fixed_links=False,
            fixed=False,
        ),
        "material": gs.materials.Rigid(
            gravity_compensation=1.0,
        ),
    }

_ASSET_FACTORIES = {
    "franka": _make_franka_urdf,
    "satellite": _make_satellite,
    "franka_merge":_make_franka_merge,
    # "satellite_part": _make_satellite_part,
    # "franka_mjcf": _make_franka_mjcf,
}

def get_asset(name: str) -> dict:
    """Build asset dict lazily after gs.init()."""
    if name not in _ASSET_FACTORIES:
        raise KeyError(f"Unknown asset '{name}'. Available: {list(_ASSET_FACTORIES)}")
    return _ASSET_FACTORIES[name]()
from configs.config_satellite import *
from configs.config_franka import *
from configs.config_starlink_manipulator import *

_ASSET_FACTORIES = {
    "franka": _make_franka_urdf,
    "satellite": _make_satellite,
    "satellite_part": _make_satellite_part,
    "franka_mjcf": _make_franka_mjcf,
    "franka_merge":_make_starlink_manipulator,
}

_ASSET_CONFIG_FACTORIES = {
    "franka": FRANKA_PARAMS,
    "franka_merge": FRANKA_S_Q_PARAMS,
}

def get_asset(name: str) -> dict:
    """Build asset dict lazily after gs.init()."""
    if name not in _ASSET_FACTORIES:
        raise KeyError(f"Unknown asset '{name}'. Available: {list(_ASSET_FACTORIES)}")
    return _ASSET_FACTORIES[name]()

def get_configs(name: str) ->dict:
    if name not in _ASSET_CONFIG_FACTORIES:
        raise KeyError(f"Unknown asset '{name}'. Available: {list(_ASSET_CONFIG_FACTORIES)}")
    return _ASSET_CONFIG_FACTORIES[name]

__all__ = [
    'get_asset',
    'get_configs',
]

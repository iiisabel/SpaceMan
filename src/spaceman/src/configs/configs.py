# Modified.
"""
Default simulation application settings.

This module defines default configurations for the simulation environment,
including scene setup, camera settings, and general application parameters.
"""

import genesis as gs

from pathlib import Path
_current_file_path = Path(__file__).resolve().parent
_ROOT_PATH = _current_file_path.parent.parent
_ASSET_PATH = _ROOT_PATH / 'src' / 'assets'
ASSET_PATH_STR = str(_ASSET_PATH)

APP_SETTINGS = {
    "seed": 0,
    "backend":gs.gpu,
    "precision": "32",
    "logging_level": "debug",
}

SCENE_SETTINGS = {
    "sim_options": gs.options.SimOptions(
        dt=3e-3,
        substeps=10,
    ),
    "rigid_options": gs.options.RigidOptions(
        gravity=(0, 0, 0),
        enable_collision=True,
        enable_self_collision=False,
        enable_adjacent_collision=False,
        constraint_timeconst=0.02,
    ),
    "viewer_options": gs.options.ViewerOptions(
        camera_pos=(-1.5, -2.0, 1.5),
        camera_lookat=(0.0, 0.0, 0.0),
        camera_fov=40,
    ),
    "vis_options": gs.options.VisOptions(
        show_world_frame=True,
        visualize_mpm_boundary=False,
    ),
    "show_viewer": True,
    "show_FPS": False,
}

TPV_CAM_SETTINGS = {
    "camera": {
        "res": (640, 480),
        "pos": (-2.5, -3.0, 2.5),
        "lookat": (0, 0, 0.5),
        "fov": 40,
        "GUI": True,
    }
}

__all__ = [
    'APP_SETTINGS',
    'SCENE_SETTINGS', 
    'TPV_CAM_SETTINGS',
]
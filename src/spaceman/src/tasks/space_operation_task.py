# This script create a customized task

import sys
import genesis as gs

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from configs.configs import APP_SETTINGS, SCENE_SETTINGS, CUSTOM_ASSETS


class QuadrotorTask:
    def __init__(self,config):
        self._name = config['robot']['robot_name']  # set robot name to
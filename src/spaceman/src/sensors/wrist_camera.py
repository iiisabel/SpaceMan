"""
可给机械臂添加手腕部相机,需要在GenesisSim()初始化之后创建
"""

import torch

# Extension APIs
import sys
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from envs.genesis_env import GenesisSim
from utils.utils import calculate_camera_pose
from controllers.backend import Backend

WRIST_CAM_SETTINGS = {
    "camera": {
        "res": (640, 480),
        "pos": (-1, -1, -1),
        "lookat": (0, 0, 0),
        "fov": 70,
        "GUI": True,
    },
    "end_effector_link": "panda_grasptarget", #"panda_hand",
}

class WristCamera(Backend):
    """
    """
    def __init__(self):
        # Add wrist camera to the scene
        self._scene = GenesisSim().scene
        self.device = GenesisSim().device
        self.datatype = torch.float32

        # Use private attribute to avoid property conflict
        self._cam = self._scene.add_camera(**WRIST_CAM_SETTINGS["camera"])
        self._robot = None
        self.end_effector = None
        self.cam_pos = None
        self.cam_lookat = None
        self.cam_up = None

    @property
    def cam(self):
        return self._cam
    
    @property
    def pos(self):
        return self.cam_pos
    
    @property
    def lookat(self):
        return self.cam_lookat
    
    def initialize(self, robot=None):
        self._robot = robot
        
        # Access Genesis robot entity through Robot wrapper (robot.robot has get_link())
        if hasattr(robot, 'robot'):
            genesis_robot = robot.robot
            self.end_effector = genesis_robot.get_link(WRIST_CAM_SETTINGS["end_effector_link"])
        else:
            raise AttributeError(f"Robot object {robot} does not have a 'robot' attribute containing the Genesis entity")
        
    def step(self):
        # self._step += 1
        # 获得robot的手腕位置姿态
        position=self.end_effector.get_pos()
        quaternion=self.end_effector.get_quat() # [qw, qx, qy, qz]
        position = torch.tensor(position, dtype=self.datatype, device=self.device)
        quaternion = torch.tensor(quaternion, dtype=self.datatype, device=self.device)  

        # 更改其camera到手腕
        self.cam_pos,self.cam_lookat, self.cam_up = calculate_camera_pose(position, quaternion)

        self.cam.set_pose(pos=self.cam_pos,lookat=self.cam_lookat, up=self.cam_up)
        
        # 返回image
        rgb, *rest = self.cam.render(
            rgb=True,
            # depth        = True,
            # segmentation = True,
        )
        return rgb

    def reset(self):
        """无需reset"""
        pass

    def stop(self):
        """无需stop"""
        pass
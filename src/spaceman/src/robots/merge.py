import sys
import torch
import copy
import numpy as np
import genesis as gs

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))

from robots.robot import Robot
from robots.franka import Franka
from envs.genesis_env import GenesisSim
from utils.twolink_state import TwoLinkState
from utils.utils import convert_dict_to_tensors
from configs.asset_configs import starlink_manipulator_FRANKA_CONFIG

class FrankaMerge(Franka):
    def __init__(
        self,
        name="franka_merge",
        sensors=[],
        backends=[]
    ):
        super(Franka, self).__init__(
            name=name,
            sensors=sensors,
            backends=backends
        )
        print("所有可用的关节名称:")
        for joint in self.robot.joints:
            print(f"  - {joint.name}")
        # ================================================
        # 只修改这一部分参数，其他全部继承父类
        # ================================================
        self.franka_name = name
        self._scene = GenesisSim().scene
        
        self.end_effector = self.robot.get_link("qf_space_manipulator_Link6")
        self.base = self.robot.get_link("starlink_base_star_link")
        
        # Need to modify FRANKA_CONFIG for starlink_combine_qf_space_manipulator
        self.config = convert_dict_to_tensors(starlink_manipulator_FRANKA_CONFIG, self.datatype, self.device)

        self.ee_state = TwoLinkState(device=self.device)
        
        self.joints_name = (
            # "root_joint",
            "qf_space_manipulator_joint1",
            "qf_space_manipulator_joint2",
            "qf_space_manipulator_joint3",
            "qf_space_manipulator_joint4",
            "qf_space_manipulator_joint5",
            "qf_space_manipulator_joint6",
            # "qf_space_manipulator_joint7",
            # "qf_space_manipulator_joint8",
            # "qf_space_manipulator_joint9",
            # "qf_space_manipulator_joint10",
            # "qf_space_manipulator_joint11",
            # "qf_space_manipulator_joint12",
        )
        
        motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.joints_name]

        self.motors_dof = motors_dof_idx[:6]
        self.fingers_dof = motors_dof_idx[6:12]
        
        self.finger_open = torch.tensor([0.04, 0.04], dtype=self.datatype, device=self.device)
        self.finger_close = torch.tensor([0.0, 0.0], dtype=self.datatype, device=self.device)
        self.gripper_state = True  # True for hand open
    
    def set_config(self, config):
        """
        重写set_config，只对Franka的DOF（索引6-14）应用配置
        卫星DOF（索引0-5）保持默认
        """
        def to_list(x):
            if isinstance(x, torch.Tensor):
                return x.detach().cpu().tolist()
            if isinstance(x, (np.ndarray, list, tuple)):
                return list(x)
            return [float(x)]
        
        kp    = to_list(config["control"]["kp"])
        kv    = to_list(config["control"]["kv"])
        frmin = to_list(config["control"]["force_range_min"])
        frmax = to_list(config["control"]["force_range_max"])
        init  = to_list(config["initial_dofs"])
        
        self.robot.set_dofs_kp(kp, dofs_idx_local=self.motors_dof + self.fingers_dof)
        self.robot.set_dofs_kv(kv, dofs_idx_local=self.motors_dof + self.fingers_dof)
        self.robot.set_dofs_force_range(frmin, frmax, dofs_idx_local=self.motors_dof + self.fingers_dof)
        self.robot.set_dofs_position(init, dofs_idx_local=self.motors_dof + self.fingers_dof)
        self.robot.set_dofs_velocity([0.0] * len(init), dofs_idx_local=self.motors_dof + self.fingers_dof)


    def control_joint_pos(self, joint_position):
        """
        Input: 
        position: [x, y, z] (torch.tensor, list, tuple or numpy array)
        quaternion: [qw, qx, qy, qz] (torch.tensor, list, tuple or numpy array)
        """
        # Convert inputs to torch tensors with correct dtype and device if needed
        if not isinstance(joint_position, torch.Tensor):
            joint_position = torch.tensor(joint_position, dtype=self.datatype, device=self.device)
        else:
            joint_position = joint_position.to(dtype=self.datatype, device=self.device)
        
        # Ensure correct shape
        if joint_position.dim() == 0:
            joint_position = joint_position.unsqueeze(0)

        
        # Ensure position has shape [3] and quat has shape [4]
        if joint_position.shape[-1] != 6:
            raise ValueError(f"Joint_position should have shape [6], got {joint_position.shape}")
        

        # Control joints' dofs
        try:
            self.robot.set_qpos(joint_position, self.motors_dof)
            
            # Optional: Get feedback for verification
            qpos_fb = self.robot.get_qpos()
            print("Current command: ", qpos_fb)
            print(f"Current joints pos : {joint_position}")
            
            return True
        except Exception as e:
            print(f"Failed to set joint positions: {e}")
            return False
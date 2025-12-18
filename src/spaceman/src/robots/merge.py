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
        
        self.end_effector = self.robot.get_link("qf_space_manipulator_2F-Body_Link")
        self.base = self.robot.get_link("starlink_base_star_link")
        
        # Need to modify FRANKA_CONFIG for starlink_combine_qf_space_manipulator
        self.config = convert_dict_to_tensors(starlink_manipulator_FRANKA_CONFIG, self.datatype, self.device)

        self.ee_state = TwoLinkState(device=self.device)
        
        self.joints_name = (
            # "root_joint",
            "qf_space_manipulator_Shoulder_link_1_yaw",
            "qf_space_manipulator_Upper_arm_Link_1_roll",
            "qf_space_manipulator_Mid_arm_Link_1_roll",
            "qf_space_manipulator_Upper_wrist_Link_1_yaw",
            "qf_space_manipulator_Upper_wrist_Link_2_roll",
            "qf_space_manipulator_2F-Body_Link_pitch", 
            # hand
            "qf_space_manipulator_Finger1_2_Link_roll",
            "qf_space_manipulator_Finger3_2_Link_roll", # all gripper joints mimic this
            "qf_space_manipulator_Finger3_1_Link_roll",
            "qf_space_manipulator_Finger1_1_Link_roll", # the only positive mutiplier
            "qf_space_manipulator_Finger4_2_Link_roll",
            "qf_space_manipulator_Finger4_1_Link_roll"
        )
        
        motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.joints_name]

        self.motors_dof = motors_dof_idx[:6]
        self.fingers_dof = motors_dof_idx[6:]

        self.config_gripper_joints = torch.tensor([-1, 1, -1, 1, -1, -1])
        
        self.finger_open = torch.tensor(-0.2, dtype=self.datatype, device=self.device).expand(6)*self.config_gripper_joints
        self.finger_close = torch.tensor(0.4, dtype=self.datatype, device=self.device).expand(6)*self.config_gripper_joints
        self.gripper_state = True  # True for hand open
    

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

    def control_gripper(self, gripper_open, gripper_value):
        """
        Control the gripper to open or close.
        Args:
            gripper_open: bool or value that can be converted to bool
                        True to open gripper, False to close
        """
        try:
            # Ensure input is boolean
            if not isinstance(gripper_open, bool):
                gripper_open = bool(gripper_open)
                if hasattr(self, 'get_logger'):
                    print(f"Converted gripper_open to boolean: {gripper_open}")
            
            # Determine target finger state
            # finger_state = self.finger_open if gripper_open else self.finger_close
            finger_state = torch.tensor(gripper_value, dtype=self.datatype, device=self.device).expand(6)*self.config_gripper_joints
            # finger_force = np.array([1.0, 1.0]) if gripper_open else np.array([-1.0, -1.0])
            # Control the gripper
            # self.robot.control_dofs_position(finger_state, self.fingers_dof)
            # self.robot.control_dofs_force(finger_force, self.fingers_dof)
            self.robot.set_qpos(finger_state, self.fingers_dof)
            
            # Update current state
            self.gripper_state = gripper_open
            
            return True
            
        except Exception as e:
            # Log the error
            error_msg = f"Failed to control gripper: {e}"
            if hasattr(self, 'get_logger'):
                print(error_msg)
            else:
                print(f"ERROR: {error_msg}")
            
            return False
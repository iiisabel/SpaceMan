import sys
import torch
import logging

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))

from robots.manipulator import Manipulator
from envs.genesis_env import GenesisSim
from utils.twolink_state import TwoLinkState
from utils.utils import convert_dict_to_tensors
from controllers.smooth_IK_solver import SmoothIKSolver

def setup_logger(name, level=logging.INFO):
    logger = logging.getLogger(name)
    
    if not logger.handlers:
        logger.setLevel(level)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        
        logger.addHandler(console_handler)
    
    return logger

class FrankaMerge(Manipulator):
    def __init__(
        self,
        name="franka_merge",
        sensors=[],
        backends=[]
    ):
        super(Manipulator, self).__init__(
            name=name,
            sensors=sensors,
            backends=backends
        )
        self.logger = setup_logger(f"FrankaMerge.{name}")
        # for joint in self.robot.joints:
        #     self.logger.debug(joint.name)
        self.franka_name = name
        self._scene = GenesisSim().scene
        
        self.end_effector = self.robot.get_link(self.params["end_effector"])
        self.base = self.robot.get_link(self.params["base"])
        
        # Need to modify FRANKA_CONFIG for starlink_combine_qf_space_manipulator
        self.config = convert_dict_to_tensors(self.params["config"], self.datatype, self.device)

        # self._base_state = SingleLinkState()
        self.ee_state = TwoLinkState(device=self.device)
        
        self.joints_name = self.params["joints"]
        
        motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.joints_name]
        # self.logger.debug(f"关节索引: {motors_dof_idx}")

        self.motors_dof = motors_dof_idx[:self.params["motor"]]
        self.fingers_dof = motors_dof_idx[self.params["motor"] : self.params["finger"]]

        self.config_gripper_joints = torch.tensor(self.params["gripper_waist"])
        
        self.finger_open = torch.tensor(self.params["finger_open"], dtype=self.datatype, device=self.device)*self.config_gripper_joints
        self.finger_close = torch.tensor(self.params["finger_close"], dtype=self.datatype, device=self.device)*self.config_gripper_joints
        self.gripper_state = True  # True for hand open
        self.logger.info("FrankaMerge robot initialization completed")

    def initialize(self):
        """After the scene is built."""
        # Initialize Franka robot's configuration
        self.set_config(self.config)

        # Initialize base and end effector's position and attitude
        self.update_state()
        self.logger.debug(f"End effector state: {self.ee_state}")

        self.IK = SmoothIKSolver(
            robot=self.robot, 
            end_effector=self.end_effector,
            smooth_factor=self.params["ik_params"]["smooth_factor"], 
            max_joint_change=self.params["ik_params"]["max_joint_change"],
        )
        self.logger.info(f"IK solver initialization completed")

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
            error_msg = f"Joint_position should have shape [6], got {joint_position.shape}"
            self.logger.error(error_msg)
            raise ValueError(error_msg)       

        # Control joints' dofs
        try:
            self.robot.set_qpos(joint_position, self.motors_dof)
            
            # Optional: Get feedback for verification
            qpos_fb = self.robot.get_qpos()
            self.logger.debug(f"Current joint feedback position: {qpos_fb}")
            self.logger.info(f"Target joint position: {joint_position}")
            
            return True
        
        except Exception as e:
            self.logger.error(f"Failed to set joint positions: {e}")
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
                self.logger.debug(f"Converted gripper_open to boolean: {gripper_open}")
            
            action = "open" if gripper_open else "close"
            self.logger.info(f"Controlling gripper {action}, gripper_value={gripper_value}")
            
            # Determine target finger state
            # finger_state = self.finger_open if gripper_open else self.finger_close
            finger_state = torch.tensor(gripper_value, dtype=self.datatype, device=self.device).expand(6)*self.config_gripper_joints
            self.logger.debug(finger_state)
            # finger_force = np.array([1.0, 1.0]) if gripper_open else np.array([-1.0, -1.0])
            # Control the gripper
            # self.robot.control_dofs_position(finger_state, self.fingers_dof)
            # self.robot.control_dofs_force(finger_force, self.fingers_dof)
            self.robot.set_qpos(finger_state, self.fingers_dof)
            
            # Update current state
            self.gripper_state = gripper_open
            self.logger.info(f"Gripper {action} control completed")
            return True
            
        except Exception as e:
            # Log the error
            error_msg = f"Failed to control gripper: {e}"
            self.logger.error(error_msg)
            return False
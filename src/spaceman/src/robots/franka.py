
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
from envs.genesis_env import GenesisSim
from utils.twolink_state import TwoLinkState
from utils.utils import convert_dict_to_tensors
from configs.asset_configs import FRANKA_CONFIG
from controllers.smooth_IK_solver import SmoothIKSolver

class Franka(Robot):
    def __init__(
        self,
        name="franka",
        sensors=[],
        backends=[]
    ):
        # Initialize the Robot object, and add robot to backends
        # Let FrankaMerge be able to have own posix
        super().__init__(
            name=name, 
            sensors=sensors,
            backends=backends
            )
        
        # Get the current world at which we want to spawn the Robot
        self.franka_name = name
        self._scene = GenesisSim().scene
        
        ### state
        self.end_effector = self.robot.get_link("panda_hand")
        self.base = self.robot.get_link("panda_link0")
        self.config = convert_dict_to_tensors(FRANKA_CONFIG, self.datatype, self.device)

        # baselink_state = self._base_state.global_state
        self.ee_state = TwoLinkState(device=self.device) # body state是相对于机械臂baselink
        
        ### controller
        self.joints_name = (
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        )

        motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.joints_name]
        self.motors_dof = motors_dof_idx[:7]
        self.fingers_dof = motors_dof_idx[7:9]

        # 
        self.finger_open = torch.tensor([0.04, 0.04],dtype=self.datatype,device=self.device)
        self.finger_close = torch.tensor([0.0, 0.0],dtype=self.datatype,device=self.device)
        self.gripper_state = True # True for hand open. [TODO] update from genesis sim

    """
    Properties
    """
    
    @property
    def end_effector_state(self):
        """The state of the robot end effector.
        Returns:
            State: The current state of the robot, i.e., position, orientation, linear and angular velocities...
        """
        return self.ee_state
    
    @property
    def ee_global_position(self):
        """The global position of the end effector.
        Returns:
            Position state (torch.tensor): 
        """
        return self.ee_state.link_child_global_state.position
    
    @property
    def ee_global_quaternion(self):
        """The global orientation of the end effector.
        Returns:
            Quaternion state (torch.tensor):
        """
        return self.ee_state.link_child_global_state.quat

    @property
    def name(self) -> str:
        """Robot name.
        Returns:
            Robot name (str): 
        """
        return self._robot_name
    

    """
    Operations
    """
    def initialize(self):
        """After the scene is built."""
        # Initialize Franka robot's configuration
        self.set_config(self.config)

        # Initialize base and end effector's position and attitude
        self.update_state()
        # print(self.ee_state)

        self.IK = SmoothIKSolver(
            robot=self.robot, 
            end_effector=self.end_effector,
            smooth_factor=0.3, 
            max_joint_change=0.05,
        )
        # There some parameters are deleted, no need to give default value.

    
    def set_config(self, config):
        """
        Apply PD, limits, and initial DOF state
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

        self.robot.set_dofs_kp(kp)
        self.robot.set_dofs_kv(kv)
        self.robot.set_dofs_force_range(frmin, frmax)
        self.robot.set_dofs_position(init)
        self.robot.set_dofs_velocity([0.0] * len(init))

    def update_state(self):
        # Update end effector's base's position and attitude
        self.ee_state.update_parent_from_global_frame(
            position=self.base.get_pos(),
            quat=self.base.get_quat(),  # quaternion: [qw, qx, qy, qz]
        )

        # Update end effector's position and attitude
        self.ee_state.update_child_from_global_frame(
            position=self.end_effector.get_pos(),
            quat=self.end_effector.get_quat(),  # quaternion: [qw, qx, qy, qz]
        )

        # Update end effector's position and attitude regard to base link
        self.ee_state.update_child_in_parent()


    def step(self):
        # Update base state and then ee state of franka
        self.update_state()

        # Call the update methods in sensors
        for sensor in self._sensors:
            sensor.step()
        
        # Call the update methods in backends
        for backend in self._backends:
            backend.step(dt=None)
    
    def control_joints(self, position, quaternion):
        """
        Input: 
        position: [x, y, z] (torch.tensor, list, tuple or numpy array)
        quaternion: [qw, qx, qy, qz] (torch.tensor, list, tuple or numpy array)
        """
        # Convert inputs to torch tensors with correct dtype and device if needed
        if not isinstance(position, torch.Tensor):
            position = torch.tensor(position, dtype=self.datatype, device=self.device)
        else:
            position = position.to(dtype=self.datatype, device=self.device)
        
        if not isinstance(quaternion, torch.Tensor):
            quaternion = torch.tensor(quaternion, dtype=self.datatype, device=self.device)
        else:
            quaternion = quaternion.to(dtype=self.datatype, device=self.device)
        
        # Ensure correct shape
        if position.dim() == 0:
            position = position.unsqueeze(0)
        if quaternion.dim() == 0:
            quaternion = quaternion.unsqueeze(0)
        
        # Ensure position has shape [3] and quat has shape [4]
        if position.shape[-1] != 3:
            raise ValueError(f"Position should have shape [3], got {position.shape}")
        if quaternion.shape[-1] != 4:
            raise ValueError(f"Quaternion should have shape [4], got {quaternion.shape}")
        
        # Compute joints' angles under global frame
        qpos = self.IK.solve(position, quaternion)
        # print("Target command: ", qpos)
        
        # Check for NaN values
        if torch.isnan(qpos).any():
            self.get_logger().warn("IK solution contains NaN values, skipping joint control")
            return False

        # Control joints' dofs
        try:
            self.robot.set_qpos(qpos[:-2], self.motors_dof)
            
            # Optional: Get feedback for verification
            # qpos_fb = self.robot.get_qpos()
            # print("Current command: ", qpos_fb)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set joint positions: {e}")
            return False
        
    def control_gripper(self, gripper_open):
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
                    self.get_logger().info(f"Converted gripper_open to boolean: {gripper_open}")
            
            # Determine target finger state
            finger_state = self.finger_open if gripper_open else self.finger_close
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
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")
            
            return False

    def show_info(self):
        print("ee body state: ", self.ee_state.link_child_in_parent)
        print("ee global state: ", self.ee_state.link_child_global_state)
        print("base global state: ", self.ee_state.link_parent_global_state)
        return super().show_info()
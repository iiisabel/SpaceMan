import numpy as np
import torch
from scipy.spatial.transform import Rotation
from typing import Optional

# Extension APIs
try:
    from .state_vectors import StateVectors
    from .singlelink_state import SingleLinkState
    from .utils import reorder_quaternion, quaternion_conjugate, quaternion_multiply
except:
    from state_vectors import StateVectors
    from singlelink_state import SingleLinkState
    from utils import reorder_quaternion

class MultiLinkState(SingleLinkState):
    """
    Stores and manages the state of a given multi-link robot in both global and body frames.
    Considering a movable base link, and the body frame is fixed to the base link.
    Automatically synchronizes all state variables when any coordinate frame is updated.
    Supports device management for GPU computation.
    """

    def __init__(self, base_link_state: StateVectors,
                 euler_order: str = "xyz", 
                 datatype = torch.float32, 
                 device: str = "cpu"):
        """
        Initialize the State object.
        Args:
            base_link_state: State object representing the base link's global state,
                            whose quaternion order follows [qx, qy, qz, qw]
            euler_order: Euler angle rotation order (default: "XYZ" for roll-pitch-yaw)
            datatype: Data type for tensors (default: torch.float32)
            device: Device for tensor storage (default: "cpu")
        """
        super().__init__(euler_order, datatype, device) 
        # self._global_state 表示ee相对于global frame的状态
        # self._body_state 表示ee相对于baselink body frame的状态
        self.update_baselink(base_link_state) 
        # self._base_link_state 表示baselink相对于global frame的状态

    @property
    def base_link_state(self):
        """Whose quaternion order [qx, qy, qz, qw]."""
        return self._base_link_state

    def update_baselink(self, base_link_state: StateVectors):
        """
        Update the base link state reference in world frame.
        Args:
            base_link_state: New base link state object, whose quaternion order is [qx, qy, qz, qw]
        """
        self._base_link_state = base_link_state
        # self._update_rotation_matrices()

        quat_base = self._base_link_state.quat # [qx, qy, qz, qw]
        if quat_base is not None: 
            self._R_base_to_global = self._base_link_state._R_mat
            self._R_global_to_base = self._base_link_state._R_mat.T

    def _synchronize_global_to_body(self):
        """
        Transform end-effector states from global frame to body frame (base_link frame).
        This accounts for the base link's global state.
        """
        # Get base link rotation matrix from global to body frame
        R_global_to_base_body = self._R_global_to_base
        
        # Transform position: body_position = R_global_to_base_body * (global_position - base_global_position)
        ee_position_global = self._global_state.position
        base_position_global = self._base_link_state.position
        relative_position_global = ee_position_global - base_position_global
        body_position = R_global_to_base_body @ relative_position_global

        self._body_state.update(position=body_position)
        
        # Transform orientation: body_quat = base_global_quat^{-1} * global_quat
        ee_quat_global = self._global_state.quat # [qx, qy, qz, qw]
        base_quat_global = self._base_link_state.quat  # [qx, qy, qz, qw]
        
        base_quat_conj = quaternion_conjugate(base_quat_global)
        body_quat = quaternion_multiply(base_quat_conj, ee_quat_global)

        self._body_state.update(quat=body_quat)
        
        # R_ee_global = Rotation.from_quat(ee_quat_global.cpu().numpy())
        # R_ee_body = self.R_obj_base.inv() * R_ee_global
        # self._body_state.update(quat=torch.tensor(R_ee_body.as_quat(), dtype=self.datatype, device=self.device))
        
        # Transform velocities
        ee_lin_vel_global = self._global_state.linear_velocity
        ee_ang_vel_global = self._global_state.angular_velocity
        base_lin_vel_global = self._base_link_state.linear_velocity
        base_ang_vel_global = self._base_link_state.angular_velocity
        
        # Linear velocity in body frame: 
        # R_global_to_base_body * (ee_lin_vel_global - base_lin_vel_global - cross(base_ang_vel_global, relative_position_global))
        cross_term = torch.cross(base_ang_vel_global, relative_position_global, dim=0)
        relative_lin_vel = ee_lin_vel_global - base_lin_vel_global - cross_term
        body_lin_vel = R_global_to_base_body @ relative_lin_vel

        self._body_state.update(linear_velocity=body_lin_vel)
        
        # Angular velocity in body frame: 
        # R_global_to_base_body * (ee_ang_vel_global - base_ang_vel_global)
        relative_ang_vel = ee_ang_vel_global - base_ang_vel_global
        body_ang_vel = R_global_to_base_body @ relative_ang_vel
        self._body_state.update(angular_velocity=body_ang_vel)

    def _synchronize_body_to_global(self):
        """
        Transform end-effector states from body frame (base_link frame) to global frame.
        This accounts for the base link's global state.
        """
        # Get base link rotation matrix from body to global frame
        R_base_body_to_global = self._R_base_to_global
        
        # Transform position: global_position = base_global_position + R_base_body_to_global * body_position
        ee_position_body = self._body_state.position
        base_position_global = self._base_link_state.position
        global_position = base_position_global + R_base_body_to_global @ ee_position_body

        self._global_state.update(position=global_position)
        
        # Transform orientation: global_quat = base_global_quat * body_quat
        ee_quat_body = self._body_state.quat # [qx, qy, qz, qw]
        R_ee_body = Rotation.from_quat(ee_quat_body.cpu().numpy())
        R_ee_global = self.R_obj_base * R_ee_body
        
        self._global_state.update(quat=torch.tensor(R_ee_global.as_quat(), dtype=self.datatype, device=self.device))
        
        # Transform velocities
        ee_lin_vel_body = self._body_state.linear_velocity
        ee_ang_vel_body = self._body_state.angular_velocity
        base_ang_vel_global = self._base_link_state.angular_velocity
        base_lin_vel_global = self._base_link_state.linear_velocity
        
        # Linear velocity in global frame: 
        # base_lin_vel_global + R_base_body_to_global @ ee_lin_vel_body + cross(base_ang_vel_global, R_base_body_to_global @ ee_position_body)
        ee_position_global_relative = R_base_body_to_global @ ee_position_body
        cross_term = torch.cross(base_ang_vel_global, ee_position_global_relative, dim=0)
        global_lin_vel = base_lin_vel_global + R_base_body_to_global @ ee_lin_vel_body + cross_term

        self._global_state.update(linear_velocity=global_lin_vel)
        
        # Angular velocity in global frame: 
        # base_ang_vel_global + R_base_body_to_global @ ee_ang_vel_body
        global_ang_vel = base_ang_vel_global + R_base_body_to_global @ ee_ang_vel_body

        self._global_state.update(angular_velocity=global_ang_vel)

    def update_from_global_frame(self, 
                               position: Optional[torch.Tensor] = None,
                               quat: Optional[torch.Tensor] = None,  # [qw,qx,qy,qz]
                               orient: Optional[torch.Tensor] = None,
                               linear_velocity: Optional[torch.Tensor] = None,
                               angular_velocity: Optional[torch.Tensor] = None):
        """
        Update end-effector state from global frame values and synchronize body frame.
        This accounts for the base link's global state. quaternion [qw,qx,qy,qz]
        """
        if quat is not None:
            quat = reorder_quaternion(quat,"xyzw")

        # Update global state using StateVectors
        self._global_state.update(
            position=position,
            quat=quat,
            orient=orient,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity
        )
        
        # Synchronize body frame states considering base link
        self._synchronize_global_to_body()
    
    def update_from_body_frame(self,
                             position: Optional[torch.Tensor] = None,
                             quat: Optional[torch.Tensor] = None,
                             orient: Optional[torch.Tensor] = None,
                             linear_velocity: Optional[torch.Tensor] = None,
                             angular_velocity: Optional[torch.Tensor] = None):
        """
        Update end-effector state from body frame values and synchronize global frame.
        This accounts for the base link's global state. quaternion [qw,qx,qy,qz]
        """
        if quat is not None:
            quat = reorder_quaternion(quat,"xyzw")

        # Update body state using StateVectors
        self._body_state.update(
            position=position,
            quat=quat,
            orient=orient,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity
        )
        
        # Synchronize global frame states considering base link
        self._synchronize_body_to_global()



# test main
if __name__ == "__main__":
    # Create base link state
    base_state = StateVectors(device="cpu")
    base_state_quat = reorder_quaternion(torch.tensor([0.707, 0.0, 0.0, 0.707]), "xyzw") # [qw, qx, qy, qz]
    base_state.update(
        position=torch.tensor([1.0, 0.0, 0.0]),
        quat=base_state_quat, # [qx, qy, qz, qw]
        linear_velocity=torch.tensor([0.1, 0.0, 0.0]),
        angular_velocity=torch.tensor([0.0, 0.0, 0.1])
    )

    # Create end-effector state with base link reference
    ee_state = MultiLinkState(base_state, device="cpu")

    # Update end-effector in global frame
    ee_state_quat = reorder_quaternion(torch.tensor([0.707, 0.0, 0.0, 0.707]), "xyzw") # [qw, qx, qy, qz]
    ee_state.update_from_global_frame(
        position=torch.tensor([1.5, 2.5, 3.5]),
        quat=torch.tensor([0.0, 0.0, 0.707, 0.707]) # [qx, qy, qz, qw]
    )
    print("End-effector global position:", ee_state.position_global)
    print("End-effector body position (relative to base):", ee_state.position_body)
    print()

    # Update end-effector in body frame
    ee_state.update_from_body_frame(
        position=torch.tensor([1.5, 2.5, 3.5]),
        orient=torch.tensor([0.0, 0.0, 0.0])
    )
    print("After body update")
    print("End-effector global position:", ee_state.position_global)
    print("End-effector body position (relative to base):", ee_state.position_body)
    print()

    ## base link moves, but end effector didnot
    base_state.update(
        position=torch.tensor([0.0, 0.0, 0.0]),
        quat=torch.tensor([1.0, 0.0, 0.0, 0.0]), # [qw,qx,qy,qz]
        linear_velocity=torch.tensor([0.0, 0.0, 0.0]),
        angular_velocity=torch.tensor([0.0, 0.0, 0.0])
    )
    ee_state.update_baselink(base_state)
    print("After base link update")
    print("End-effector global position:", ee_state.position_global)
    print("End-effector body position (relative to base):", ee_state.position_body)

import math
import torch
from scipy.spatial.transform import Rotation
from typing import Optional, Union


class StateVectors:
    """
    Stores and manages the state vectors of a given robot, including position, orientation and velocity.
    Automatically synchronizes all orientation variables. All quaternion orders follow [qx, qy, qz, qw].
    Supports device management for GPU computation.
    """

    def __init__(self, euler_order: str = "XYZ", datatype = torch.float32, device: str = "cpu"):
        """
        Initialize the State object.
        Args:
            euler_order: Euler angle rotation order (default external rotation : "XYZ" for roll-pitch-yaw)
        """
        self.euler_order = euler_order
        self.device = torch.device(device)
        self.datatype = datatype

        # Initialize all state variables
        self._initialize_states()

    def _initialize_states(self):
        """Initialize all state variables with zeros"""
        # Global frame states
        self._position = torch.tensor([0.0, 0.0, 0.0], dtype=self.datatype, device=self.device)
        self._quat = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=self.datatype, device=self.device)  # [qx, qy, qz, qw]
        self._orient = torch.tensor([0.0, 0.0, 0.0], dtype=self.datatype, device=self.device) # -pi ~ pi
        self._R_mat = torch.eye(3, dtype=self.datatype, device=self.device) # identity matrix

        self._linear_velocity = torch.tensor([0.0, 0.0, 0.0], dtype=self.datatype, device=self.device)
        self._angular_velocity = torch.tensor([0.0, 0.0, 0.0], dtype=self.datatype, device=self.device)

    @property
    def position(self):
        """position: [x, y, z]"""
        return self._position.clone()
    
    @property
    def quat(self):
        """quaternion: [qx, qy, qz, qw]"""
        return self._quat.clone()
    
    @property
    def orient(self):
        """rotation order: self.euler_order"""
        return self._orient.clone()
    
    @property
    def R_mat(self):
        """rotation matrix, transfer from other frame to this frame."""
        return self._R_mat.clone()

    @property
    def linear_velocity(self):
        return self._linear_velocity.clone()
    
    @property
    def angular_velocity(self):
        return self._angular_velocity.clone()

    def update(self, 
               position: Optional[torch.Tensor] = None,
               orient: Optional[torch.Tensor] = None,
               quat: Optional[torch.Tensor] = None,
               linear_velocity: Optional[torch.Tensor] = None,
               angular_velocity: Optional[torch.Tensor] = None):
        """
        Update state variables with automatic synchronization of orientation representations.
        Args:
            position: Position vector [x, y, z] in global frame
            orient: Euler angles [roll, pitch, yaw] in global frame
            quat: Quaternion [qx, qy, qz, qw] in global frame
            linear_velocity: Linear velocity vector [vx, vy, vz] in global frame
            angular_velocity: Angular velocity vector [ωx, ωy, ωz] in global frame
        Raises:
            ValueError: If both orient and quat are provided simultaneously
        """
        # Update position if provided
        if position is not None:
            self._position = position.to(dtype=self.datatype, device=self.device)
        
        # Update orientation representations with priority handling
        if orient is not None and quat is not None:
            raise ValueError("Cannot provide both orient and quat simultaneously. "
                           "Please provide only one orientation representation.")
        
        if orient is not None:
            # Update from Euler angles
            self._update_rotation_matrices_from_euler(orient)
        elif quat is not None:
            # Update from quaternion
            self._update_rotation_matrices_from_quat(quat)
        
        # Update velocities if provided
        if linear_velocity is not None:
            self._linear_velocity = linear_velocity.to(dtype=self.datatype, device=self.device)
        
        if angular_velocity is not None:
            self._angular_velocity = angular_velocity.to(dtype=self.datatype, device=self.device)

    def add(self, 
            position: Optional[torch.Tensor] = None,
            orient: Optional[torch.Tensor] = None,
            quat: Optional[torch.Tensor] = None,
            linear_velocity: Optional[torch.Tensor] = None,
            angular_velocity: Optional[torch.Tensor] = None):
        """
        Add incremental values to state variables with automatic synchronization of orientation representations.
                
        Args:
            position: Position increment [dx, dy, dz] in global frame
            orient: Euler angle increment [d_roll, d_pitch, d_yaw] in global frame
            quat: Quaternion increment [dqx, dqy, dqz, dqw] in global frame
            linear_velocity: Linear velocity increment [dvx, dvy, dvz] in global frame
            angular_velocity: Angular velocity increment [dωx, dωy, dωz] in global frame
            
        Raises:
            ValueError: If both orient and quat are provided simultaneously
        """
        # Add position increment if provided
        if position is not None:
            position_increment = position.to(dtype=self.datatype, device=self.device)
            self._position = self._position + position_increment
        
        # Add orientation increments with priority handling
        if orient is not None and quat is not None:
            raise ValueError("Cannot provide both orient and quat simultaneously. "
                        "Please provide only one orientation representation.")
        
        if orient is not None:
            # Add to Euler angles and update other representations
            orient_increment = orient.to(dtype=self.datatype, device=self.device)
            new_orient = self._orient + orient_increment
            self._update_rotation_matrices_from_euler(new_orient)
        elif quat is not None:
            # Add to quaternion and update other representations
            quat_increment = quat.to(dtype=self.datatype, device=self.device)
            new_quat = self._quat + quat_increment
            # Normalize quaternion after addition to maintain unit quaternion property
            new_quat = new_quat / torch.norm(new_quat)
            self._update_rotation_matrices_from_quat(new_quat)
        
        # Add velocity increments if provided
        if linear_velocity is not None:
            linear_velocity_increment = linear_velocity.to(dtype=self.datatype, device=self.device)
            self._linear_velocity = self._linear_velocity + linear_velocity_increment
        
        if angular_velocity is not None:
            angular_velocity_increment = angular_velocity.to(dtype=self.datatype, device=self.device)
            self._angular_velocity = self._angular_velocity + angular_velocity_increment

    def _update_rotation_matrices_from_quat(self, quat: torch.Tensor):
        """Update rotation matrices from global quaternion [qx, qy, qz, qw]"""
        # Check if quaternion is not zero
        if torch.allclose(quat, torch.zeros_like(quat)):
            quat_normalized = torch.tensor([0.0, 0.0, 0.0, 1.0]) # Use identity quaternion if input is zero
        else:
            quat_normalized = quat / torch.norm(quat) # Normalize the quaternion to ensure it's a unit quaternion

        self._quat = quat_normalized.to(dtype=self.datatype, device=self.device)

        # Update orientation angles
        R = Rotation.from_quat(quat_normalized.cpu().numpy(), scalar_first=False)  # [qx, qy, qz, qw]
        self._orient = torch.tensor(R.as_euler(self.euler_order, degrees=False), dtype=self.datatype, device=self.device)
        self._R_mat = torch.tensor(R.as_matrix(), dtype=self.datatype, device=self.device) # transfer to this frame from others

    def _update_rotation_matrices_from_euler(self, euler: torch.Tensor):
        """Update rotation matrices from global Euler angles"""
        # -pi ~ pi
        euler = torch.remainder(euler + math.pi, 2*math.pi) - math.pi
        self._orient = euler.to(dtype=self.datatype, device=self.device)

        # Update quaternions
        R = Rotation.from_euler(self.euler_order, euler.cpu().numpy(), degrees=False)
        self._quat = torch.tensor(R.as_quat(), dtype=self.datatype, device=self.device)
        self._R_mat = torch.tensor(R.as_matrix(), dtype=self.datatype, device=self.device) # transfer to this frame from others

    def to(self, device: Union[str, torch.device]):
        """Move all state tensors to the specified device"""
        device = torch.device(device)
        if device == self.device:
            return self
        
        self.device = device
        
        # Move all tensors to the new device
        for attr_name in dir(self):
            if attr_name.startswith('_') and not attr_name.startswith('__'):
                attr = getattr(self, attr_name)
                if isinstance(attr, torch.Tensor):
                    setattr(self, attr_name, attr.to(device))
        
        return self

    def __repr__(self):
        return (f"State(euler_order='{self.euler_order}')\n"
                f"Position: {self.position}\n"
                f"Orientation: {self.orient}\n"
                f"Quaternion (xyzw): {self.quat}\n"
                f"Linear_Velocity: {self.linear_velocity}\n"
                f"Angular_Velocity: {self.angular_velocity}\n")
    

if __name__ == '__main__':
    state = StateVectors()

    pos = torch.tensor([1.0, 0.0, 0.0])
    orient = torch.tensor([1.57, 0.0, 0.0])
    state.update(position=pos, orient=orient)
    
    print(state)
    print(state.R_mat)
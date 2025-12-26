
import sys
import torch
import logging
import genesis as gs
import math
import numpy as np

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from envs.genesis_env import GenesisSim
from configs.asset_configs import *
from utils.singlelink_state import SingleLinkState

def setup_logger(name, level=logging.INFO):
    logger = logging.getLogger(name)
    
    if not logger.handlers:
        logger.setLevel(level)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        
        logger.addHandler(console_handler)
    
    return logger

class Robot:
    def __init__(
        self,
        name="franka",
        sensors=[],
        backends=[],
    ):
        self.logger = setup_logger(f"Robot.{name}")
        self.logger.info(f"Initializing robot: {name}")

        # Get the current scene 
        self._robot_name = name
        self._scene = GenesisSim().scene
        self.device = GenesisSim().device
        self.datatype = GenesisSim().datatype
        
        # entity, config = parse_asset_config(ASSETS, name)
        try:
            asset = get_asset(name)
            self.params = get_configs(name)
            self.pid_params = get_pid(name)
            self.robot = self._scene.add_entity(**asset)
            self.logger.info(f"Robot asset loaded and added to scene: {name}")
        except Exception as e:
            self.logger.error(f"Failed to load robot asset: {e}")
            raise

        self.pid = PIDController(
            P=self.pid_params["P"],
            I=self.pid_params["I"],
            D=self.pid_params["D"],
            setpoint=self.pid_params["setpoint"],
            dt=self.pid_params["dt"],
            output_limits=self.pid_params["limits"]
        )

        self._links = []
        self._get_links()

        # Initialize the base's position and orientation
        self._base_state = SingleLinkState(device=self.device)
        self.logger.debug("Base state initialized")

        # --------------------------------------------------------------------
        # -------------------- Add sensors to the robot --------------------
        # --------------------------------------------------------------------
        self._sensors = sensors
        self.logger.info(f"Adding {len(self._sensors)} sensors to robot")

        for sensor in self._sensors:
            sensor.initialize(self)
        self.logger.info("All sensors initialized")

        # --------------------------------------------------------------------
        # -------------------- Add control backends to the robot -----------
        # --------------------------------------------------------------------
        self._backends = backends
        self.logger.info(f"Adding {len(self._backends)} backends to robot")

        # Initialize the backends
        for backend in self._backends:
            backend.initialize(self)
        self.logger.info("All backends initialized")

        self.logger.info(f"Robot {name} initialization completed")

    """
    Properties
    """
    
    @property
    def base_state(self):
        """The state of the robot.

        Returns:
            State: The current state of the robot, i.e., position, orientation, linear and angular velocities...
        """
        return self._base_state
    
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
        self.logger.debug("Robot initialize method called")
        return

    def step(self):
        self.logger.debug("Robot step method called")
        return
    
    def stop(self):
        self.logger.info("Stopping robot sensors and backends")
        
        for sensor in self._sensors:
            sensor.stop()
        for backend in self._backends:
            backend.stop()
            
        self.logger.info("All robot components stopped")
        return

    def reset(self):
        self.logger.info("Robot reset method called")
        return

    def _get_links(self):
        self._links = {
            link.name: link.idx
            for link in getattr(self.robot, "links", [])
        }
        self.logger.debug(f"Found {len(self._links)} links in robot")

    def show_info(self):
        self.logger.info(f"Robot Name: {self._robot_name}")
        self.logger.info(f"Available Links: {list(self._links.keys())}")
        self.logger.info(f"Link Indices: {list(self._links.values())}")
        self.logger.info(f"Total Links: {len([link for link in self.robot.links])}")
        return

    def apply_force(self, force=None, torque=None, link_name='base_link'):
        """
        Method that will apply a force on the rigidbody, on the part specified in the 'body_part' at its relative position
        given by 'pos' (following a FLU) convention. 

        Args:
            force (list): A 3-dimensional vector of floats with the force [Fx, Fy, Fz] on the body axis of the vehicle according to a FLU convention.
            pos (list): _description_. Defaults to [0.0, 0.0, 0.0].
            body_part (str): . Defaults to "/body".
        """
        if not self._links:
            self._get_links()
        link_idx = self._links.get(link_name)
        if link_idx is None:
            error_msg = f"Unknown link '{link_name}'. Available: {list(self._links.keys())}"
            self.logger.error(error_msg)
            raise KeyError(error_msg)

        force_tensor = self._format_wrench_input(force)
        torque_tensor = self._format_wrench_input(torque)

        # Get the handle of the rigidbody that we will apply the force to
        if force_tensor is not None:
            self._scene.sim.rigid_solver.apply_links_external_force(force=force_tensor, links_idx=link_idx)
            self.logger.debug(f"Applied force {force_tensor.squeeze().tolist()} to link '{link_name}' (ID: {link_idx})")
        if torque_tensor is not None:
            self._scene.sim.rigid_solver.apply_links_external_torque(torque=torque_tensor, links_idx=link_idx)
            self.logger.debug(f"Applied torque {torque_tensor.squeeze().tolist()} to link '{link_name}' (ID: {link_idx})")

    def _format_wrench_input(self, vec):
        if vec is None:
            return None

        if not isinstance(vec, torch.Tensor):
            vec = torch.tensor(vec, dtype=self.datatype, device=self.device)
        else:
            vec = vec.to(dtype=self.datatype, device=self.device)

        if vec.numel() != 3:
            error_msg = f"Wrench inputs must contain exactly 3 elements, got shape {tuple(vec.shape)}"
            self.logger.error(error_msg)
            raise ValueError(error_msg)
        
        vec = vec.reshape(1, 3)

        return vec


class PIDController:
    def __init__(self, P, I, D, setpoint, dt=0.01, output_limits=None):
        """
        PyTorch GPU版本的6自由度PID控制器
        """
        self.device = GenesisSim().device
        self.datatype = GenesisSim().datatype
        self.dt = dt

        self.Kp = torch.tensor(P, device=self.device, dtype=self.datatype)
        self.Ki = torch.tensor(I, device=self.device, dtype=self.datatype)
        self.Kd = torch.tensor(D, device=self.device, dtype=self.datatype)
        self.setpoint = torch.tensor(setpoint, device=self.device, dtype=self.datatype)
        self.output_limits = self._parse_output_limits(output_limits)
        
        self.prev_error = torch.zeros(6, device=self.device, dtype=self.datatype)
        self.integral = torch.zeros(6, device=self.device, dtype=self.datatype)

        self.init_P = self.Kp.clone()
        self.init_I = self.Ki.clone()
        self.init_D = self.Kd.clone()
        self.init_setpoint = self.setpoint.clone()

    def _parse_output_limits(self, output_limits):
        if output_limits is None:
            return None
        elif isinstance(output_limits, (int, float)):
            return [(float(-output_limits), float(output_limits))] * 6
        elif isinstance(output_limits, (list, tuple)):
            if len(output_limits) == 6:
                limits = []
                for limit in output_limits:
                    if limit is None:
                        limits.append((-float('inf'), float('inf')))
                    elif isinstance(limit, (int, float)):
                        limits.append((float(-limit), float(limit)))
                    elif isinstance(limit, (list, tuple)) and len(limit) == 2:
                        limits.append((float(limit[0]), float(limit[1])))
                return limits

    def _wrap_error(self, error, setpoint, current):
        wrapped_error = error.clone()
        
        for i in range(3, 6):
            angle_diff = setpoint[i] - current[i]
            wrapped_error[i] = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        return wrapped_error
    
    def _apply_output_limits(self, output):
        if self.output_limits is None:
            return output
        
        limited_output = output.clone()
        for i in range(6):
            min_val, max_val = self.output_limits[i]
            limited_output[i] = torch.clamp(output[i], min_val, max_val)
        
        return limited_output

    def control(self, pos, orien):
        """
        计算控制输出
        
        Args:
            pos: 当前位置 [x, y, z]
            orien: 当前姿态 [roll, pitch, yaw]
            
        Returns:
            pos_control: 位置控制输出 [x, y, z]
            orien_control: 姿态控制输出 [roll, pitch, yaw]
        """
        if not isinstance(pos, torch.Tensor):
            pos = torch.tensor(pos, device=self.device, dtype=self.datatype)
        else:
            pos = pos.to(self.device)
            
        if not isinstance(orien, torch.Tensor):
            orien = torch.tensor(orien, device=self.device, dtype=self.datatype)
        else:
            orien = orien.to(self.device)
        
        current_state = torch.cat([pos, orien])        
        
        raw_error = self.setpoint - current_state
        error = self._wrap_error(raw_error, self.setpoint, current_state)

        self.integral = self.integral + error * self.dt

        P = self.Kp * error
        I = self.Ki * self.integral
        error_diff = error - self.prev_error

        for i in range(3, 6):
            if torch.abs(error_diff[i]) > math.pi:
                if error_diff[i] > 0:
                    error_diff[i] = error_diff[i] - 2 * math.pi
                else:
                    error_diff[i] = error_diff[i] + 2 * math.pi
        
        D = self.Kd * error_diff / self.dt
        
        self.prev_error = error.clone()
        output = P + I + D

        output = self._apply_output_limits(output)

        pos_control = output[:3]
        orien_control = output[3:]
        
        return pos_control, orien_control
    
    def reset(self):
        self.prev_error.zero_()
        self.integral.zero_()
        self.Kp = self.init_P.clone()
        self.Ki = self.init_I.clone()
        self.Kd = self.init_D.clone()
        self.setpoint = self.init_setpoint.clone()

    def _wrap_to_pi(self, angles):
        wrapped = (angles + math.pi) % (2 * math.pi) - math.pi
        return wrapped
    
    def update_setpoint(self, setpoint):
        self.setpoint = torch.tensor(setpoint, device=self.device, dtype=self.datatype)
        for i in range(3, 6):
            if self.setpoint[i].abs() > math.pi:
                self.setpoint[i] = self._wrap_to_pi(self.setpoint[i])
    
    def update_gains(self, P=None, I=None, D=None):
        if P is not None:
            self.Kp = torch.tensor(P, device=self.device, dtype=self.datatype)
        if I is not None:
            self.Ki = torch.tensor(I, device=self.device, dtype=self.datatype)
        if D is not None:
            self.Kd = torch.tensor(D, device=self.device, dtype=self.datatype)

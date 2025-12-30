
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
from controllers.backend import Backend
from controllers.pid import PIDController
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
        sensors=None,
        backends=None,
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

        # 处理传感器 - 确保是列表
        if sensors is None:
            sensors = []
        elif not isinstance(sensors, list):
            sensors = [sensors]
        self._sensors = sensors
        
        # 处理后端 - 确保是列表
        if backends is None:
            backends = []
        elif not isinstance(backends, list):
            backends = [backends]
            
        # 如果没有提供后端，创建默认的PIDController
        if not backends:
            pid_controller = PIDController(
                P=self.pid_params["P"],
                I=self.pid_params["I"],
                D=self.pid_params["D"],
                setpoint=self.pid_params["setpoint"],
                dt=self.pid_params["dt"],
                output_limits=self.pid_params["limits"],
                name=f"{name}_pid"
            )
            backends = [pid_controller]
            
        self._backends = backends
        for backend in self._backends:
            if isinstance(backend, PIDController):
                self.pid = backend
                break

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
    
    def step(self, dt: float = None):
        self.logger.debug("Robot step method called")
                
        self._update_state()

        for sensor in self._sensors:
            sensor.step()

        for backend in self._backends:
            if hasattr(backend, 'update_state') and self._base_state:
                backend.update_state(self._base_state.position_global, self._base_state.quat_global)

            backend.step()

            if hasattr(backend, 'input_reference'):
                control_output = backend.input_reference()
                self._apply_control_output(control_output=control_output, link_name=self.params["base"])
        
        return self._base_state
    
    def _update_state(self):
        self._base_state.update_from_global_frame(
            position=self.robot.get_link(self.params["base"]).get_pos(),
            quat = self.robot.get_link(self.params["base"]).get_quat(),
        )
    
    def _apply_control_output(self, control_output, link_name='base_link'):
        if control_output is None:
            return
            
        if isinstance(control_output, dict):
            force = control_output.get('position')
            torque = control_output.get('orientation')
            
            if force is not None or torque is not None:
                self.apply_force(force=force, torque=torque, link_name=link_name)
                # print(force, torque)

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
        # print("Yes")
        
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

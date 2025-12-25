
import sys
import torch
import genesis as gs

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from envs.genesis_env import GenesisSim
from configs.asset_configs import get_asset, get_configs
from utils.singlelink_state import SingleLinkState


class Robot:
    def __init__(
        self,
        name="franka",
        sensors=[],
        backends=[],
    ):
        # Get the current scene 
        self._robot_name = name
        self._scene = GenesisSim().scene
        self.device = GenesisSim().device
        self.datatype = GenesisSim().datatype
        
        # entity, config = parse_asset_config(ASSETS, name)
        asset = get_asset(name)
        self.params = get_configs(name)
        self.robot = self._scene.add_entity(**asset)
        self._links = []
        self._get_links()

        # Initialize the base's position and orientation
        self._base_state = SingleLinkState(device=self.device)
        # baselink_state = self._base_state.global_state

        # --------------------------------------------------------------------
        # -------------------- Add sensors to the robot --------------------
        # --------------------------------------------------------------------
        self._sensors = sensors

        for sensor in self._sensors:
            sensor.initialize(self)

        # --------------------------------------------------------------------
        # -------------------- Add control backends to the robot -----------
        # --------------------------------------------------------------------
        self._backends = backends

        # Initialize the backends
        for backend in self._backends:
            backend.initialize(self)


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
        # Initialize robot's configuration
        # self.set_config(self.config)
        return

    def step(self):
        return
    
    def stop(self):
        for sensor in self._sensors:
            sensor.stop()
        for backend in self._backends:
            backend.stop()
        return

    def reset(self):
        return

    def _get_links(self):
        self._links = {
            link.name: link.idx
            for link in getattr(self.robot, "links", [])
        }

    def show_info(self):
        # self._get_links()
        print(self._robot_name)
        print(list(self._links.keys()))
        print(list(self._links.values()))
        print([link for link in self.robot.links])
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
            raise KeyError(f"Unknown link '{link_name}'. Available: {list(self._links.keys())}")

        force_tensor = self._format_wrench_input(force)
        torque_tensor = self._format_wrench_input(torque)

        # Get the handle of the rigidbody that we will apply the force to
        if force_tensor is not None:
            self._scene.sim.rigid_solver.apply_links_external_force(force=force_tensor, links_idx=link_idx)
        if torque_tensor is not None:
            self._scene.sim.rigid_solver.apply_links_external_torque(torque=torque_tensor, links_idx=link_idx)
            # print(f"robot {self._robot_name} applies torque of {torque_tensor}, at link ID {link_idx}")

    def _format_wrench_input(self, vec):
        if vec is None:
            return None

        if not isinstance(vec, torch.Tensor):
            vec = torch.tensor(vec, dtype=self.datatype, device=self.device)
        else:
            vec = vec.to(dtype=self.datatype, device=self.device)

        if vec.numel() != 3:
            raise ValueError(f"Wrench inputs must contain exactly 3 elements, got shape {tuple(vec.shape)}")

        vec = vec.reshape(1, 3)

        return vec

        




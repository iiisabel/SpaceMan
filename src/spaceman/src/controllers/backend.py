# __all__ = ["Backend"]

from abc import ABC, abstractmethod


class Backend(ABC):
    """
    This class defines the templates for the communication and control backend. Every robot can have at least one backend
    at the same time. Every timestep, the methods 'update_state' and 'update_sensor' are called to update the data produced
    by the simulation, i.e. for every time step the backend will receive teh current state of the robot and its sensors. 
    Additionally, the backend must provide a method named 'input_reference' which will be used by the robot simulation
    to know the desired angular velocities to apply to the rotors of the robot. The method 'update' is called on every
    physics step and can be use to implement some logic or send data to another interface (such as PX4 through mavlink or ROS2).
    The methods 'start', 'stop' and 'reset' are callbacks that get called when the simulation is started, stoped and reset as the name implies.
    """

    def __init__(self):
        """Initialize the Backend class
        """
        self._robot = None

    """
     Properties
    """
    @property
    def robot(self):
        """A reference to the robot associated with this backend.

        Returns:
            robot: A reference to the robot associated with this backend.
        """
        return self._robot

    def initialize(self, robot):
        """A method that can be invoked when the simulation is starting to give access to the control backend 
        to the entire robot object. Even though we provide update_sensor and update_state callbacks that are called
        at every physics step with the latest robot state and its sensor data, having access to the full robot
        object may prove usefull under some circumstances. This is nice to give users the possibility of overiding
        default robot behaviour via this control backend structure.

        Args:
            robot (robot): A reference to the robot that this sensor is associated with
        """
        self._robot = robot
    
    def update_sensor(self, sensor_type: str, data):
        """Method that when implemented, should handle the receival of sensor data

        Args:
            sensor_type (str): A name that describes the type of sensor
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass

    def input_reference(self):
        """Method that when implemented, should return a list of desired angular velocities to apply to the robot rotors
        """
        return []

    # @abstractmethod
    def update_state(self, state):
        """Method that when implemented, should handle the receival of the state of the robot using this callback

        Args:
            state (State): The current state of the robot.
        """
        pass

    @abstractmethod
    def step(self, dt: float):
        """Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    # @abstractmethod
    def start(self):
        """Method that when implemented should handle the begining of the simulation of robot
        """
        pass

    @abstractmethod
    def stop(self):
        """Method that when implemented should handle the stopping of the simulation of robot
        """
        pass

    @abstractmethod
    def reset(self):
        """Method that when implemented, should handle the reset of the robot simulation to its original state
        """
        pass

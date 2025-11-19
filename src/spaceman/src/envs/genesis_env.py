import sys
import torch
from threading import Lock
import genesis as gs

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from configs.configs import APP_SETTINGS, SCENE_SETTINGS, TPV_CAM_SETTINGS
from sensors.timer import SimpleTimer
from utils.utils import with_camera, generate_filename

class GenesisSim:
    """
    GenesisSim is a singleton class (there is only one object instance at any given time) that is responsible 
    for orchestrating the entire simulation: it sets up the scene, constructs robots and map objects 
    (the actors), and provides a set of APIs to manage the simulation world.
    """
    # The object instance
    _instance = None
    _is_initialized = False

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):
        """
        Initialize the GenesisSim environment.
        """
        # If we already have an instance of the PegasusInterface, do not overwrite it!
        if GenesisSim._is_initialized:
            return
        
        GenesisSim._is_initialized = True

        # Initialize the simulation
        gs.init(**APP_SETTINGS)
        try:
            print(f"[DEBUG] APP_SETTINGS: seed={APP_SETTINGS.get('seed', None)} precision={APP_SETTINGS.get('precision', None)}")
        except Exception:
            pass

        self.scene = gs.Scene(**SCENE_SETTINGS)
        self.rigid = self.scene.sim.rigid_solver
        self.viewer = self.scene.viewer
        self.device = torch.cuda.current_device()
        self.datatype = torch.float32
        
        # Add third-person view (TPV) camera
        if with_camera(TPV_CAM_SETTINGS):
            self.cam = self.scene.add_camera(**TPV_CAM_SETTINGS["camera"])
        
        # plane = self.scene.add_entity(
        #     gs.morphs.Plane(
        #         pos=(0, 0, -100.0),
        #     ),
        # )
    
    """
    Properties
    """
    @property
    def time(self):
        return self.timer.time

    """
    Operations
    """
    def start(self):
        print("[DEBUG] Building scene...")
        self.scene.build()
        print("[DEBUG] Scene built.")
        
        self.timer = SimpleTimer()
        # If scene camera is set
        if with_camera(TPV_CAM_SETTINGS):
            self.cam.start_recording()

    def step(self):
        self.scene.step()
        # If scene camera is set
        if with_camera(TPV_CAM_SETTINGS):
            self.cam.render()
        
    
    def stop(self):
        if with_camera(TPV_CAM_SETTINGS):
            save_file = generate_filename(prefix="video", extension="mp4", folder_path="recordings")
            self.cam.stop_recording(save_to_filename=save_file, fps=60)
            print("Recorded video is saved to ", save_file)


    def reset(self):
        self.scene.reset()
        self.timer.reset()

    def attach_robot_to_object(self, robot, obj):
        # create fixed contraints
        # robot_idx = robot.get_link("hand").idx
        # object_idx = obj.get_link("attachment").idx
        # self.rigid.add_weld_constraint(robot_idx, object_idx)
        self.scene.link_entities(obj, robot, "base", "panda_link0")


    def __new__(cls):
        """Allocates the memory and creates the actual simulation object is not instance exists yet. Otherwise,
        returns the existing instance of the simulation class.

        Returns:
            cls: the single instance of this class
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of this class
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(GenesisSim, cls).__new__(cls)

        return cls._instance
    
    def __del__(self):
        """Destructor for the object. Destroys the only existing instance of this class."""
        GenesisSim._instance = None
        GenesisSim._is_initialized = False



if __name__ == "__main__":
    # 
    GS = GenesisSim()
    
    GS.start()
    # for i in range(1000):
    #     # print("current simulation time: ", GS.time)
    #     GS.update()
    
    while True:
        GS.step()
        if not GS.viewer.is_alive():
            print("Viewer window has been closed.")
            break

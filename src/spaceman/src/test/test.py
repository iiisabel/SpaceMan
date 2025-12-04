#
import sys
import torch
import genesis as gs

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from envs.genesis_env import GenesisSim

def is_finite_tensor(x) -> bool:
    if isinstance(x, (list, tuple)):
        try:
            x = torch.tensor(x, dtype=torch.float32)
        except Exception:
            return False
    if not isinstance(x, torch.Tensor):
        return False
    return torch.isfinite(x).all().item()

def check_link_pose(entity, link_name, device="cuda:0"):
    try:
        link = entity.get_link(link_name)
    except Exception as e:
        print(f"[DEBUG] Link '{link_name}' not found on entity: {e}")
        return False
    pos = torch.tensor(link.get_pos(), dtype=torch.float32, device=device)
    quat = torch.tensor(link.get_quat(), dtype=torch.float32, device=device)
    ok = torch.isfinite(pos).all().item() and torch.isfinite(quat).all().item()
    if not ok:
        print(f"[DEBUG][NaN] link={link_name} pos={pos.cpu().tolist()} quat={quat.cpu().tolist()}")
    return ok

def list_links(entity):
    try:
        names = [l.name for l in entity.links]
        print(f"[DEBUG] Links: {names}")
        return names
    except Exception as e:
        print(f"[DEBUG] Could not list links: {e}")
        return []

def main():
    GS = GenesisSim()

    # create franka keyboard operator
    from controllers.keyboard_controller import KeyboardController
    controller = KeyboardController()

    from sensors.wrist_camera import WristCamera
    wrist_camera = WristCamera()

    from robots.franka import Franka
    franka = Franka(name="franka",sensors=[], backends=[])

    if False:
        # runtime patch: give small positive inertials to massless links
        try:
            from utils.patch_urdf import fix_urdf_inertials
            urdf_path = (current_file_path.parent / "assets" / "urdf" / "satellite" / "urdf" / "satellite.urdf").resolve()
            fix_urdf_inertials(urdf_path, min_mass=1e-2, min_inertia=1e-4, links_to_fix=["left_Link", "attachment"])
            print(f"[DEBUG] URDF inertials patched at {urdf_path}")
        except Exception as e:
            print(f"[DEBUG] URDF inertial patch skipped: {e}")

    from robots.robot import Robot
    # satellite = Robot(name="satellite")
    # satellite_part = Robot(name="satellite_part")

    # 
    GS.start()
    franka.initialize()
    # satellite.initialize()
    # satellite.show_info()
    # try:
    #     GS.scene.link_entities(satellite.robot, franka.robot, "attachment", "panda_link0")
    # except:
    #     pass

    while True:
        # satellite.apply_force(force=[0.0,0.0,10.0], torque=[0.0,0.0,1000.0],link_name='base')
        franka.step()
        GS.step()
        if not GS.viewer.is_alive(): # 
            print("Viewer window has been closed.")
            break

    GS.stop()

if __name__ == "__main__":
    main()

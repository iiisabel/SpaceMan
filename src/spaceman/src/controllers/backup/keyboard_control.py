import threading
import time
import math
import torch
from pynput import keyboard
import copy

# Extension APIs
import sys
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from utils.state_vectors import StateVectors
from controllers.backend import Backend

class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.last_pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        # 定义增量值
        self.d_pos = 0.05  # 每次按键移动的距离
        self.d_rot = 0.05  # 每次按键旋转的角度（弧度）
        #预设姿态
        self.pose1_orient = torch.tensor([0.0, 0.0, 0.0])
        self.pose2_orient = torch.tensor([math.pi/2, 0.0, 0.0])
        self.pose3_orient = torch.tensor([-math.pi/2, math.pi/4, 0.0])

    def start(self):
        "启动键盘监听线程。"
        self.listener.start()
        self._print_controls()

    def stop(self):
        "停止键盘监听线程，并等待其结束。"
        self.listener.stop()
        self.listener.join()

    def on_press(self, key: keyboard.Key):
        "当键被按下时调用，将键添加到集合中。"
        with self.lock:
            self.pressed_keys.add(key)

    def on_release(self, key: keyboard.Key):
        "当键被释放时调用，将键从集合中移除。"
        with self.lock:
            self.pressed_keys.discard(key)

    def get_pressed_keys(self):
        with self.lock:
            return self.pressed_keys.copy()

    def get_newly_pressed_keys(self):
        with self.lock:
            current_pressed = self.pressed_keys.copy()
        newly_pressed = current_pressed - self.last_pressed_keys
        self.last_pressed_keys = current_pressed
        return newly_pressed
    
    def _print_controls(self):
        print("位置控制:")
        print("方向键 ↑/↓ - 沿X轴前进/后退")
        print("方向键 ←/→ - 沿Y轴左移/右移")
        print("右Shift/右ctrl - 沿Z轴上升/下降")
        print()
        print("旋转控制:")
        print("j/n - 绕X轴正转/逆转")
        print("b/m- 绕Y轴正转/逆转")
        print("k/, - 绕Z轴正转/逆转")
        print()
        print("预设姿态 (单击):")
        print("1 - 姿态1   2 - 姿态2   3 - 姿态3")
        print()
        print("夹爪控制:")
        print("空格键 - 切换夹爪状态 (张开/闭合)")
        print()
        print("退出 (单击):")
        print("ESC - 退出程序")


class KeyboardControl(Backend):
    def __init__(self):
        self.keyboard_client = KeyboardDevice()
        self._state = StateVectors()
        self.gripper_open = True
        self.running = False
        self._robot = None

    def initialize(self, robot=None):
        self._robot = robot
        self.keyboard_client.start()
        self.running = True
        # print(f"夹爪初始状态: {'张开' if self.gripper_open else '闭合'}")

    def stop(self):
        self.keyboard_client.stop()
        self.running = False

    def step(self, dt: float):
        """执行一步动作，供外部循环调用"""
        pressed_keys = self.keyboard_client.get_pressed_keys()
        newly_pressed_keys = self.keyboard_client.get_newly_pressed_keys()

        # --- 退出（单击触发） ---
        if keyboard.Key.esc in newly_pressed_keys:
            print("执行: 退出程序。")
            self.running = False
            return

        # --- 预设姿态（单击触发） ---
        if keyboard.KeyCode.from_char("1") in newly_pressed_keys:
            print("执行: 切换到姿态1。")
            self._state.update(orient=self.keyboard_client.pose1_orient)
        elif keyboard.KeyCode.from_char("2") in newly_pressed_keys:
            print("执行: 切换到姿态2。")
            self._state.update(orient=self.keyboard_client.pose2_orient)
        elif keyboard.KeyCode.from_char("3") in newly_pressed_keys:
            print("执行: 切换到姿态3。")
            self._state.update(orient=self.keyboard_client.pose3_orient)

        # --- 夹爪状态切换（单击触发） ---
        if keyboard.Key.space in newly_pressed_keys:
            self.gripper_open = not self.gripper_open
            if self.gripper_open:
                print("执行: 夹爪张开。")
            else:
                print("执行: 夹爪闭合。")

        # --- 位置控制（长按触发） ---
        if keyboard.Key.up in pressed_keys:
            self._state.add(position=torch.tensor([self.keyboard_client.d_pos, 0.0, 0.0]))
        if keyboard.Key.down in pressed_keys:
            self._state.add(position=torch.tensor([-self.keyboard_client.d_pos, 0.0, 0.0]))
        if keyboard.Key.left in pressed_keys:
            self._state.add(position=torch.tensor([0.0, self.keyboard_client.d_pos, 0.0]))
        if keyboard.Key.right in pressed_keys:
            self._state.add(position=torch.tensor([0.0, -self.keyboard_client.d_pos, 0.0]))
        if keyboard.Key.shift_r in pressed_keys:
            self._state.add(position=torch.tensor([0.0, 0.0, self.keyboard_client.d_pos]))
        if keyboard.Key.ctrl_r in pressed_keys:
            self._state.add(position=torch.tensor([0.0, 0.0, -self.keyboard_client.d_pos]))

        # --- 旋转控制（长按触发） ---
        if keyboard.KeyCode.from_char("j") in pressed_keys:
            self._state.add(orient=torch.tensor([self.keyboard_client.d_rot, 0.0, 0.0]))
        if keyboard.KeyCode.from_char("n") in pressed_keys:
            self._state.add(orient=torch.tensor([-self.keyboard_client.d_rot, 0.0, 0.0]))
        if keyboard.KeyCode.from_char("b") in pressed_keys:
            self._state.add(orient=torch.tensor([0.0, self.keyboard_client.d_rot, 0.0]))
        if keyboard.KeyCode.from_char("m") in pressed_keys:
            self._state.add(orient=torch.tensor([0.0, -self.keyboard_client.d_rot, 0.0]))
        if keyboard.KeyCode.from_char("k") in pressed_keys:
            self._state.add(orient=torch.tensor([0.0, 0.0, self.keyboard_client.d_rot]))
        if keyboard.KeyCode.from_char(",") in pressed_keys:
            self._state.add(orient=torch.tensor([0.0, 0.0, -self.keyboard_client.d_rot]))

        # 打印当前状态以进行实时反馈
        # print(f"\rPosition: {self._state.position.cpu().numpy()}, Orientation: {self._state.orient.cpu().numpy()}", end="")

        return self._state.position, self._state.quat, self.gripper_open

    def reset(self):
        return

    def update_state(self, state):
        """Input current state of robot's end effector.
        Args:
            state: (StateVectors) e.g., self.ee_state.body_state
        """
        self._state = copy.deepcopy(state)
        # 
        


if __name__ == "__main__":
    action = KeyboardControl()
    action.initialize()

    try:
        while action.running:
            # action.update_state()
            action.step(None)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n程序被中断。")
    finally:
        action.stop()
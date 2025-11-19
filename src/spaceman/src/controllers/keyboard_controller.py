import torch
import pygame
import time
import copy
from pygame.locals import *

# Extension APIs
import sys
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from utils.state_vectors import StateVectors
from controllers.backend import Backend

class KeyboardController(Backend):
    def __init__(self):
        # Initialize the state of end effector
        self._state = StateVectors() 
        self.gripper_open = True
        self.running = False # running state
        self._robot = None

        # moving step length
        self.d_pos = 0.05  # 位置移动步长 (米)
        self.d_rot = 0.1   # 旋转步长 (弧度)
        
        
    def initialize(self, robot=None):
        self._robot = robot
        # 初始化pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Keyboard Control")
        self.font = pygame.font.Font(None, 24)
        # 
        self.running = True
        # self._show_info()

    def stop(self):
        pygame.quit()
        self.running = False

    def step(self, dt: float):
        """运行键盘控制循环"""
        
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.running = False
                
                # --- 位置控制（长按触发） ---
                elif event.key == K_UP: # 向前
                    self._state.add(position=torch.tensor([self.d_pos, 0.0, 0.0]))
                elif event.key == K_DOWN: # 向后
                    self._state.add(position=torch.tensor([-self.d_pos, 0.0, 0.0]))
                elif event.key == K_LEFT: # 向左
                    self._state.add(position=torch.tensor([0.0, self.d_pos, 0.0]))
                elif event.key == K_RIGHT: # 向右
                    self._state.add(position=torch.tensor([0.0, -self.d_pos, 0.0]))
                elif event.key == K_RSHIFT: # 向上
                    self._state.add(position=torch.tensor([0.0, 0.0, self.d_pos]))
                elif event.key == K_RCTRL: # 向下
                    self._state.add(position=torch.tensor([0.0, 0.0, -self.d_pos]))

                # --- 旋转控制（长按触发） ---
                elif event.key == K_j:
                    self._state.add(orient=torch.tensor([self.d_rot, 0.0, 0.0]))
                elif event.key == K_n:
                    self._state.add(orient=torch.tensor([-self.d_rot, 0.0, 0.0]))
                elif event.key == K_b:
                    self._state.add(orient=torch.tensor([0.0, self.d_rot, 0.0]))
                elif event.key == K_m:
                    self._state.add(orient=torch.tensor([0.0, -self.d_rot, 0.0]))
                elif event.key == K_k:
                    self._state.add(orient=torch.tensor([0.0, 0.0, self.d_rot]))
                elif event.key == K_COMMA:
                    self._state.add(orient=torch.tensor([0.0, 0.0, -self.d_rot]))

                # --- 夹爪状态切换（单击触发） ---
                elif event.key == K_SPACE:
                    self.gripper_open = not self.gripper_open
                    # if self.gripper_open:
                    #     print("执行: 夹爪张开。")
                    # else:
                    #     print("执行: 夹爪闭合。")
        
        # 清屏
        self.screen.fill((0, 0, 0))
        
        # 显示当前位置
        pos_text = self.font.render(f"Position: {self._state.position}", True, (255, 255, 255))
        orient_text = self.font.render(f"Orientation: {self._state.orient}", True, (255, 255, 255))
        gripper_text = self.font.render(f"Gripper: {'Open' if self.gripper_open else 'Closed'}", True, (255, 255, 255))
        move_help_text = self.font.render("Direction up/dn/lf/rg/shf/ctl: Move", True, (255, 255, 255))
        rotate_help_text = self.font.render("j/n/b/m/k/,: Rotate, R/F: Gripper, ESC: Quit", True, (255, 255, 255))

        self.screen.blit(pos_text, (10, 10))
        self.screen.blit(orient_text, (10, 40))
        self.screen.blit(gripper_text, (10, 70))
        self.screen.blit(move_help_text, (10, 100))
        self.screen.blit(rotate_help_text, (10, 130))
        
        pygame.display.flip()
        
        # 模拟一步
        # time.sleep(0.01)  # 控制循环速度
        
        return self._state.position, self._state.quat, self.gripper_open
    
    def reset(self):
        return

    def update_state(self, state):
        """Input current state of robot's end effector.
        Args:
            state: (StateVectors) e.g., self.ee_state.body_state
        """
        self._state = copy.deepcopy(state)

    def _show_info(self):
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

# 使用示例
if __name__ == "__main__":
    # 假设已经初始化了franka, scene, motors_dof和fingers_dof
    # franka = ... 
    # scene = ...
    robot_state = StateVectors() 
    # robot_state.update(position=,orient=)
    # 创建控制器实例并运行
    controller = KeyboardController()
    controller.initialize()
    try:
        while controller.running:
            # action.update_state()
            controller.step()
    except KeyboardInterrupt:
        print("\n程序被中断。")
    finally:
        controller.stop()
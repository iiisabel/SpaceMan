# coding=utf-8
# A ROS2 python node for a remote keyboard controller
# for 6 DoFs manipulator
import sys
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import pygame
from pygame.locals import *

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
# PoseStamped includes, pose.orientation.(x,y,z,w), pose.position.(x,y,z)
from std_msgs.msg import Float64


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # create end effector listener
        self.ee_sub = self.create_subscription(
            PoseStamped,
            '/ee_pose_broadcaster/global_pose',
            self.ee_pose_callback,
            10)
        
        # create gripper listener
        self.gripper_sub = self.create_subscription(
            Float64,
            '/ee_pose_broadcaster/gripper_state',
            self.gripper_callback,
            10)

        # create end effector pose control publisher
        self.ee_pub = self.create_publisher(
            PoseStamped,
            '/ee_control/pose_command',
            10
        )

        # create gripper state control publisher
        self.gripper_pub = self.create_publisher(
            Float64,
            '/ee_control/gripper_command',
            10
        )
        self.initialize()

        # 使用定时器
        self.control_timer = self.create_timer(0.05, self.control_update)  # 20Hz 控制更新
        

    def initialize(self):
        # === 初始化pygame ===
        pygame.init()
        self.screen = pygame.display.set_mode((420, 260))
        pygame.display.set_caption("Keyboard Arm Control")
        self.font = pygame.font.Font(None, 26)

        # === 位姿参数 ===
        self.init_pos = np.array([-1.38, 1.53, 0.48]) # 初始末端位置
        self.pos = self.init_pos.copy()
        self.init_quat = np.array([1.0, 0.0, 0.0, 0.0]) # 初始末端姿态 [w,x,y,z]
        self.rot = R.from_quat(self.init_quat, scalar_first=True)

        self.gripper = 1.0  # 1=open, 0=close

        # moving step length
        self.d_pos = 0.03  # 位置移动步长 (米)
        self.d_rot = 0.1   # 旋转步长 (弧度)

        # === 位姿边界 ===
        # self.limit_min = [0.1, -0.3, 0.06]  # 下限
        # self.limit_max = [0.45, 0.3, 0.3]  # 上限

        # === 预设点 ===
        self.presets = {
            pygame.K_1: self.init_pos,
        }

        # === 键盘状态 ===
        self.long_press = {
            'W': False, 'S': False, 'A': False, 'D': False, 'R': False, 'F': False,
            'UP': False, 'DOWN': False, 'LEFT': False, 'RIGHT': False, 'RSHIFT': False, 'RCTRL': False,
        }

        self.short_press = {'SPACE': True}
        
        # 创建按键映射字典
        self.key_mapping = {
            # translation
            'W': pygame.K_w,
            'S': pygame.K_s,
            'A': pygame.K_a,
            'D': pygame.K_d,
            'R': pygame.K_r,
            'F': pygame.K_f,
            # rotation
            'UP': pygame.K_UP,
            'DOWN': pygame.K_DOWN,
            'LEFT': pygame.K_LEFT,
            'RIGHT': pygame.K_RIGHT,
            'RSHIFT': pygame.K_RSHIFT,
            'RCTRL': pygame.K_RCTRL,
            #
            'SPACE': pygame.K_SPACE,
        }
        self.long_press_last = copy.copy(self.long_press)

        # 添加线程锁保护共享状态
        self.state_lock = threading.Lock()

        self.running = True
        self.get_key = False
        self.clock = pygame.time.Clock()

        print("控制说明：")
        print("W/S=X轴前后, A/D=Y轴左右, R/F=Z轴上下")
        print("↑/↓=X轴旋转, ←/→=Y轴旋转, Rshift/Rctrl=Z轴旋转")
        print("空格=开关夹爪")
        print("数字 1/2/3 为预设姿态, ESC 退出")

    def ee_pose_callback(self, msg):
        """
        接收末端执行器位姿并更新位置和旋转
        
        Args:
            msg: geometry_msgs.msg.PoseStamped 消息
        """
        try:
            # 提取位置信息 [x, y, z]
            new_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            
            # 提取四元数信息并转换为 [w, x, y, z] 格式
            new_quat = np.array([
                msg.pose.orientation.w,  # w 分量
                msg.pose.orientation.x,  # x 分量
                msg.pose.orientation.y,  # y 分量
                msg.pose.orientation.z   # z 分量
            ])
            
            # 更新位置和旋转
            self.pos = new_pos
            self.rot = R.from_quat(new_quat, scalar_first=True)  # scipy 的 from_quat 期望 [x, y, z, w] 顺序
            
        except Exception as e:
            # 错误处理
            error_msg = f"Error updating EE pose: {e}"
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")

    def gripper_callback(self, msg):
        """
        接收夹爪状态并更新 self.gripper
        Args:
            msg: std_msgs.msg.Float64
        """
        try:
            # 使用锁保护状态更新
            with self.state_lock:
                # 提取浮点数值
                gripper_value = msg.data
                
                # 使用阈值将浮点数转换为整数值
                # 约定：> 0.5 为打开 (1)，<= 0.5 为关闭 (0)
                self.gripper = 1.0 if gripper_value > 0.5 else 0.0
        except Exception as e:
            # 错误处理
            error_msg = f"Error updating gripper state: {e}"
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")

    def clear_key_states(self):
        keys = pygame.key.get_pressed()
        for key_name in self.long_press.keys():
            if key_name in self.key_mapping and not keys[self.key_mapping[key_name]]:
                self.long_press[key_name] = False

    def handle_events(self):
        """键盘事件监听"""
        events = pygame.event.get()
        
        key_events = [e for e in events if e.type in (pygame.QUIT, pygame.KEYDOWN, pygame.KEYUP)]
        self.get_key = False
        if key_events:
            # print("events: ", events)
            # print("key_events: ", key_events)
            self.get_key = True
        else:
            return
        
        for event in key_events:
            if event.type == pygame.QUIT:
                self.exit_program()

            # 按键按下
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.exit_program()
                #
                elif event.key == pygame.K_w: self.long_press['W'] = True
                elif event.key == pygame.K_s: self.long_press['S'] = True
                elif event.key == pygame.K_a: self.long_press['A'] = True
                elif event.key == pygame.K_d: self.long_press['D'] = True
                elif event.key == pygame.K_r: self.long_press['R'] = True
                elif event.key == pygame.K_f: self.long_press['F'] = True
                #
                elif event.key == pygame.K_UP: self.long_press['UP'] = True
                elif event.key == pygame.K_DOWN: self.long_press['DOWN'] = True
                elif event.key == pygame.K_LEFT: self.long_press['LEFT'] = True
                elif event.key == pygame.K_RIGHT: self.long_press['RIGHT'] = True
                elif event.key == pygame.K_RSHIFT: self.long_press['RSHIFT'] = True
                elif event.key == pygame.K_RCTRL: self.long_press['RCTRL'] = True

                # --- 夹爪状态切换（单击触发） ---
                elif event.key == pygame.K_SPACE:
                    self.short_press['SPACE'] = not self.short_press['SPACE']

                # === 预设点触发 ===
                elif event.key in self.presets:
                    self.pos = self.presets[event.key].copy()
                    print(f"→ 回到预设点 {pygame.key.name(event.key)}:{self.pos}")

            # 按键释放
            elif event.type == pygame.KEYUP:
                #
                if event.key == pygame.K_w: self.long_press['W'] = False
                elif event.key == pygame.K_s: self.long_press['S'] = False
                elif event.key == pygame.K_a: self.long_press['A'] = False
                elif event.key == pygame.K_d: self.long_press['D'] = False
                elif event.key == pygame.K_r: self.long_press['R'] = False
                elif event.key == pygame.K_f: self.long_press['F'] = False
                #
                elif event.key == pygame.K_UP: self.long_press['UP'] = False
                elif event.key == pygame.K_DOWN: self.long_press['DOWN'] = False
                elif event.key == pygame.K_LEFT: self.long_press['LEFT'] = False
                elif event.key == pygame.K_RIGHT: self.long_press['RIGHT'] = False
                elif event.key == pygame.K_RSHIFT: self.long_press['RSHIFT'] = False
                elif event.key == pygame.K_RCTRL: self.long_press['RCTRL'] = False

    def update_motion(self):
        """根据长按状态更新位置和夹爪"""
        # 使用锁保护状态更新
        with self.state_lock:
            # === 末端位姿 ===
            if self.long_press['W']: self.pos[0] += self.d_pos
            if self.long_press['S']: self.pos[0] -= self.d_pos
            if self.long_press['A']: self.pos[1] += self.d_pos
            if self.long_press['D']: self.pos[1] -= self.d_pos
            if self.long_press['R']: self.pos[2] += self.d_pos
            if self.long_press['F']: self.pos[2] -= self.d_pos
            #
            if self.long_press['UP']: self.rot = R.from_euler("x", self.d_rot) * self.rot
            if self.long_press['DOWN']: self.rot = R.from_euler("x", -self.d_rot) * self.rot
            if self.long_press['LEFT']: self.rot = R.from_euler("y", self.d_rot) * self.rot
            if self.long_press['RIGHT']: self.rot = R.from_euler("y", -self.d_rot) * self.rot
            if self.long_press['RSHIFT']: self.rot = R.from_euler("z", self.d_rot) * self.rot
            if self.long_press['RCTRL']: self.rot = R.from_euler("z", -self.d_rot) * self.rot

            # === 限幅 ===
            # self.pos = np.clip(self.pos, self.limit_min, self.limit_max)

            # === 夹爪 ===
            if self.short_press['SPACE']:
                self.gripper = 1.0
                # print("Gripper Open (1)")
            elif not self.short_press['SPACE']:
                self.gripper = 0.0      
                # print("Gripper Close (0)")

        # === 夹爪 ===
        self.long_press_last = copy.copy(self.long_press)
        self.clear_key_states()
        # print("long press dict: ", self.long_press)

    # 
    def _publish_ee_pose(self):
        """
        发布末端执行器位姿到 /ee_control/pose_command
        """
        try:
            # 使用锁保护状态读取
            with self.state_lock:
                current_pos = self.pos.copy()
                current_rot = self.rot

            # 创建 PoseStamped 消息
            pose_msg = PoseStamped()
            
            # 设置消息头
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            # pose_msg.header.frame_id = "world"  # 或者你的基础坐标系
            
            # 设置位置
            pose_msg.pose.position.x = float(current_pos[0])
            pose_msg.pose.position.y = float(current_pos[1])
            pose_msg.pose.position.z = float(current_pos[2])
            
            # 将旋转转换为四元数 (注意顺序转换)
            # current_rot 是 Rotation 对象，as_quat() 返回 [w, x, y, z]
            quat = current_rot.as_quat(scalar_first=True)
            # 转换为 ROS2 的四元数
            pose_msg.pose.orientation.w = float(quat[0])
            pose_msg.pose.orientation.x = float(quat[1])
            pose_msg.pose.orientation.y = float(quat[2])
            pose_msg.pose.orientation.z = float(quat[3])
            
            # 发布消息
            self.ee_pub.publish(pose_msg)
            
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"Error publishing EE pose: {e}")
            else:
                print(f"ERROR publishing EE pose: {e}")

    def _publish_gripper_state(self):
        """
        发布夹爪状态到 /ee_control/gripper_command
        """
        try:
            # 使用锁保护状态读取
            with self.state_lock:
                current_gripper = self.gripper

            # 创建 Float64 消息
            gripper_msg = Float64()
            
            # 设置夹爪状态值
            gripper_msg.data = float(current_gripper)
            
            # 发布消息
            self.gripper_pub.publish(gripper_msg)
            
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"Error publishing gripper state: {e}")
            else:
                print(f"ERROR publishing gripper state: {e}")


    def draw_ui(self):
        """在窗口中显示当前状态"""
        self.screen.fill((0, 0, 0))
        text_pos = self.font.render(
            f"Position: x={self.pos[0]:.2f}, y={self.pos[1]:.2f}, z={self.pos[2]:.2f}",
            True, (255, 255, 255)
        )
        text_grip = self.font.render(
            f"Gripper: {'Open(1)' if self.gripper == 1 else 'Closed(0)'}",
            True, (255, 255, 255)
        )
        text_hint = self.font.render("WASD/up down + r l control, 1/2/3 presets, ESC exit", True, (200, 200, 200))

        self.screen.blit(text_pos, (20, 60))
        self.screen.blit(text_grip, (20, 100))
        self.screen.blit(text_hint, (20, 130))
        pygame.display.flip()

    def exit_program(self):
        print("退出控制。")
        self.running = False
        # 取消定时器
        if hasattr(self, 'control_timer'):
            self.control_timer.cancel()

        pygame.quit()
        sys.exit()

    def destroy_node(self):
        """重写销毁方法，确保资源清理"""
        self.exit_program()
        super().destroy_node()

    def update(self):
        """更新函数"""
        self.update_motion()
        #
        self._publish_ee_pose() # 发布末端执行器位姿
        self._publish_gripper_state() # 发布夹爪状态
        self.draw_ui()
        # get key
        self.handle_events()

    def control_update(self):
        """
        定时器回调：更新当前状态并执行控制逻辑
        """
        if not self.running:
            self.exit_program()
            return
            
        try:
            self.update()
        except Exception as e:
            print(f"Error in control_update: {e}")
            self.exit_program()
        except KeyboardInterrupt:
            # 关机
            print("Program is shutting down!")
            self.exit_program()


def main():
    rclpy.init()
    node = KeyboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


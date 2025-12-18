# coding=utf-8
# A ROS2 python node for a remote keyboard controller
# for 6 DoFs manipulator
import sys
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
# PoseStamped includes, pose.orientation.(x,y,z,w), pose.position.(x,y,z)
from std_msgs.msg import Float64

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr

class MasterEndControl(Node):
    def __init__(self):
        super().__init__('master_ee_contorl_node')

        self.bot = InterbotixManipulatorXS(
            robot_model='wx250s',
            group_name='arm',
            gripper_name='gripper',
        )

        # create end effector listener
        self.joints_sub = self.create_subscription(
            JointState,
            '/wx250s/joint_states',
            self.joint_state_callback,
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
        self.control_timer = self.create_timer(0.05, self.update)  # 20Hz 控制更新
        
        

    def initialize(self):
        self.joint_names = []
        self.joint_positions = []
        self.end_effector_pos = []
        self.end_effector_quat=[]
        self.gripper = 1.0
        self.T_sb = None


    def joint_state_callback(self, msg):
        try:
            self.joint_names = msg.name
            self.joint_positions = msg.position
            self.log_joint_states()
            self.end_effector_position_FK()
            self.get_gripper_state()
        except Exception as e:
            self.get_logger().error(f"Joint state callback error: {e}")

    def log_joint_states(self):
        if not hasattr(self, 'joint_names') or not self.joint_names:
            return
        info_lines = [f""]
        for i, name in enumerate(self.joint_names):
            line = f"  {name}: "
            if i < len(self.joint_positions):
                line += f"pos={self.joint_positions[i]:.3f}rad "
            info_lines.append(line)
        self.get_logger().info('\n'.join(info_lines))

    def end_effector_position_FK(self):
        self.T_sb = mr.FKinSpace(
            self.bot.arm.robot_des.M, self.bot.arm.robot_des.Slist, self.joint_positions[:6]
        )
        self.get_logger().info(f"T_sb Matrix:")
        for i in range(4):
            self.get_logger().info(
                f"[{self.T_sb[i,0]:8.4f} {self.T_sb[i,1]:8.4f} "
                f"{self.T_sb[i,2]:8.4f} {self.T_sb[i,3]:8.4f}]"
            )

        # Pos
        self.end_effector_pos = self.T_sb[:3, 3]  
        self.get_logger().info(
            f"End-effector Position: X={self.T_sb[0,3]:.3f}, "
            f"Y={self.T_sb[1,3]:.3f}, "
            f"Z={self.T_sb[2,3]:.3f}"
        )
        
        # Quat
        rot_matrix = self.T_sb[:3, :3]
        R_extra = R.from_euler('yx', [90, 0], degrees=True)
        rot_matrix_adjusted = (R_extra * R.from_matrix(rot_matrix)).as_matrix()
        self.end_effector_rot = R.from_matrix(rot_matrix_adjusted)

    def get_gripper_state(self):
        try:
            gripper_value = self.joint_positions[6]
            if self.gripper == 1.0 and gripper_value<=-1.37:
                self.gripper = 0.0
            elif self.gripper == 0.0 and gripper_value >= -0.7:
                self.gripper = 1.0
        except Exception as e:
            error_msg = f"Error updating gripper state: {e}"
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")
        self.get_logger().info(
            f"gripper state = " f"Open" if self.gripper==1.0 else "Close"
        )
        # -0.683 -1.382
        
    def _publish_ee_pose(self):
        """
        发布末端执行器位姿到 /ee_control/pose_command
        """
        try:
            current_pos = self.end_effector_pos.copy()
            current_rot = self.end_effector_rot

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

    def update(self):
        """更新函数"""
        self._publish_ee_pose() # 发布末端执行器位姿
        self._publish_gripper_state() # 发布夹爪状态

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = MasterEndControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

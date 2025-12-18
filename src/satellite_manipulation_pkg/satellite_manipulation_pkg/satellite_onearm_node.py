# A ROS2 python node for a simulation task of 
# a satellite with a single manipulator (Franka)

import sys
import genesis as gs
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header, Float64

# Extension APIs
from pathlib import Path
current_file_path = Path(__file__).resolve().parent
# current_file_path = "/home/lianxin/Projects/SpaceMan_ws/src/satellite_manipulation_pkg/satellite_manipulation_pkg"
workspace_src_path = current_file_path.parent.parent
# workspace_src_path = "/home/lianxin/Projects/SpaceMan_ws"
spaceman_path = workspace_src_path.joinpath("spaceman/src")
sys.path.append(str(spaceman_path))


class SatelliteOneArm(Node):
    def __init__(self):
        super().__init__('satellite_onearm_node')
        self.initialize()
        
        # create end effector pose publisher
        self.ee_state_pub = self.create_publisher(
            PoseStamped,
            '/ee_pose_broadcaster/global_pose',
            10
        )

        # create gripper state publisher
        self.gripper_state_pub = self.create_publisher(
            Float64,
            '/ee_pose_broadcaster/gripper_state',
            10
        )

        # create control action listener
        self.ee_command_sub = self.create_subscription(
            PoseStamped,
            '/ee_control/pose_command',
            self.ee_command_callback,
            10)
        
        # create control action listener
        self.gripper_command_sub = self.create_subscription(
            Float64,
            '/ee_control/gripper_command',
            self.gripper_command_callback,
            10)
        
        # 使用定时器
        self.simulation_timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 状态变量
        self.simulation_running = True

        # threading.Thread(target=self.main_thread, daemon=True).start()
        
    def initialize(self):
        from envs.genesis_env import GenesisSim
        self.gsim = GenesisSim()

        from robots.franka import Franka
        self.franka = Franka(name="franka",sensors=[], backends=[])

        from robots.robot import Robot
        self.satellite = Robot(name="satellite")
        
        # try:
        #     self.gsim.scene.link_entities(self.satellite.robot, self.franka.robot, "attachment", "panda_link0")
        # except:
        #     pass

        self.device = self.gsim.device
        self.datatype = self.gsim.datatype

        # simulation env start
        self.gsim.start()
        self.franka.initialize()
        self.satellite.initialize()



    def main_thread(self):
        """genesis simulation main loop"""
        self.counter = 0.0
        self.rate = self.create_rate(10, self.get_clock())  # 设置循环频率为1Hz
        while self.gsim.viewer.is_alive():
            #
            self.franka.step()
            self.gsim.step()

            self.end_effector_update()
            self.gripper_update()

            self.rate.sleep()
        # shuting down
        print("Viewer window has been closed.")
        self.gsim.stop()

    def timer_callback(self):
        """模拟步进函数，由定时器触发"""
        if not self.gsim.viewer.is_alive():
            self.get_logger().info("Viewer window has been closed.")
            self.simulation_running = False
            self.simulation_timer.cancel()
            self.gsim.stop()
            return
        # main
        try:
            # 执行模拟步进
            self.franka.step()
            self.gsim.step()
            
            # 更新状态并发布
            self.end_effector_update()
            self.gripper_update()
            
        except Exception as e:
            self.get_logger().error(f"Simulation step error: {e}")

    def end_effector_update(self):
        """
        将PyTorch tensor格式的位置和四元数转换为ROS2 PoseStamped消息并发布
        Args:
            position_tensor: torch.tensor [x, y, z]
            quat_tensor: torch.tensor [qw, qx, qy, qz]
        """
        position_tensor = self.franka.ee_global_position
        quat_tensor = self.franka.ee_global_quaternion

        # 确保数据在CPU上并且是numpy数组格式
        if position_tensor.is_cuda:
            position_np = position_tensor.cpu().numpy()
        else:
            position_np = position_tensor.numpy()
        
        if quat_tensor.is_cuda:
            quat_np = quat_tensor.cpu().numpy()
        else:
            quat_np = quat_tensor.numpy()
        
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        
        # 设置header
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置 (x, y, z)
        pose_msg.pose.position.x = float(position_np[0])
        pose_msg.pose.position.y = float(position_np[1])
        pose_msg.pose.position.z = float(position_np[2])
        
        # 设置四元数 (注意ROS2使用[x, y, z, w]顺序)
        # 你的tensor是[qw, qx, qy, qz]，需要转换为[x, y, z, w]
        pose_msg.pose.orientation.w = float(quat_np[0])  # qw
        pose_msg.pose.orientation.x = float(quat_np[1])  # qx
        pose_msg.pose.orientation.y = float(quat_np[2])  # qy  
        pose_msg.pose.orientation.z = float(quat_np[3])  # qz
        
        
        # 发布消息
        self.ee_state_pub.publish(pose_msg)
        
    def gripper_update(self):
        gripper_state = self.franka.gripper_state

        # hand open: True->1.0, hand close: False->0.0
        if hasattr(gripper_state, 'item'):  # 如果是 torch.tensor
            gripper_value = gripper_state.item()
        else:  # 如果是 Python bool
            gripper_value = gripper_state
        
        # 创建Float64消息
        gripper_msg = Float64()
        gripper_msg.data = 1.0 if gripper_value else 0.0
        
        # 发布消息
        self.gripper_state_pub.publish(gripper_msg)
    
    def ee_command_callback(self, msg):
        """
        将接收到的 PoseStamped 消息用来控制末端位姿 
        Args:
            msg: geometry_msgs.msg.PoseStamped 消息
        """
        try:
            # 提取位置信息并转换为 torch tensor [x, y, z]
            position = torch.tensor([
                msg.pose.position.x,
                msg.pose.position.y, 
                msg.pose.position.z
            ], dtype=self.datatype, device=self.device)
            
            # 提取四元数信息并转换为 torch tensor [w, x, y, z]
            quaternion = torch.tensor([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ], dtype=self.datatype, device=self.device)
            
            # 调用你的控制函数
            self.franka.control_joints(position, quaternion)
            
        except Exception as e:
            # 错误处理
            error_msg = f"Error processing PoseStamped command: {e}"
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")
    
    def gripper_command_callback(self, msg):
        """
        将接收到的 Float64 消息转换为布尔值并控制夹爪
        Args:
            msg: std_msgs.msg.Float64 消息
        """
        try:
            # 提取浮点数值
            gripper_value = msg.data
            
            # 将浮点数转换为布尔值
            # 通常约定：> 0.5 为打开，<= 0.5 为关闭
            gripper_open = gripper_value > 0.5
            
            # 调用夹爪控制函数
            self.franka.control_gripper(gripper_open)
            
        except Exception as e:
            # 错误处理
            error_msg = f"Error processing gripper command: {e}"
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            else:
                print(f"ERROR: {error_msg}")

    def destroy_node(self):
        """重写销毁方法，确保资源清理"""
        if hasattr(self, 'simulation_timer'):
            self.simulation_timer.cancel()
        if hasattr(self, 'gsim'):
            self.gsim.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = SatelliteOneArm()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    main()

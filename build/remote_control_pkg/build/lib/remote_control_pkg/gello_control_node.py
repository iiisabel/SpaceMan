# coding=utf-8
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

# GELLO 核心导入
from gello.agents.gello_agent import GelloAgent

# ROS2 消息
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

@dataclass
class Args:
    """GELLO node configuration"""
    agent: str = "gello"
    hz: int = 100
    # 已更新为您提供的实际端口值
    gello_port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTABJ0GT-if00-port0"
    verbose: bool = True
    start_joints: Optional[Tuple[float, ...]] = None

    def __post_init__(self):
        if self.start_joints is not None:
            self.start_joints = np.array(self.start_joints)

class GelloControl(Node):
    def __init__(self):
        super().__init__('gello_control_node')
        
        # 1. 初始化配置
        self.args = Args()
        
        # 2. 初始化 GELLO 硬件代理
        self.get_logger().info(f"Connecting to GELLO on port: {self.args.gello_port}")
        try:
            # GelloAgent 内部会通过端口映射表找到对应的 Dynamixel 配置
            self.agent = GelloAgent(
                port=self.args.gello_port,
                start_joints=self.args.start_joints
            )
        except Exception as e:
            self.get_logger().error(f"Cannot initialize GELLO hardware: {e}")
            sys.exit(1)

        # 3. 创建 ROS2 发布者
        # 发布到指令 Topic，队列深度为 10
        self.joint_pub = self.create_publisher(JointState, '/wx250s/joint_states', 10)

        # 4. 创建高频定时器
        timer_period = 1.0 / self.args.hz
        self.timer = self.create_timer(timer_period, self.publish_callback)
        
        self.get_logger().info(f"GELLO publisher node has started, frequency: {self.args.hz}Hz")

    def publish_callback(self):
        """定时器回调：读取硬件并发布消息"""
        try:
            # 直接从 GELLO 硬件读取当前关节状态
            # 由于 GelloAgent.act 内部仅使用 self._robot.get_joint_state()，传空字典即可
            action = self.agent.act(obs={})
            
            # 封装并发布 JointState 消息
            self._send_ros_msg(action)

            # 终端回显
            if self.args.verbose:
                # 打印前 3 个关节和末端夹爪的值（示例）
                gripper_val = action[-1] if len(action) > 0 else 0.0
                print(f"\r[GELLO Output] Joints: {np.round(action[:-1], 3)} | Gripper: {action[-1]:.4f}", end="")

        except Exception as e:
            self.get_logger().error(f"控制循环出错: {e}")

    def _send_ros_msg(self, action: np.ndarray):
        """构造并发送 JointState 消息"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 针对 wx250s 的典型关节命名
        # 如果关节数量不符，会自动回退到 joint_N 命名方式
        standard_names = [
            'waist', 'shoulder', 'elbow', 
            'forearm_roll', 'wrist_angle', 'wrist_rotate', 'gripper'
        ]
        
        if len(action) == len(standard_names):
            msg.name = standard_names
        else:
            msg.name = [f'joint_{i}' for i in range(len(action))]
            
        msg.position = action.tolist()
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GelloControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User quits. System is shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
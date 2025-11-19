import os
import math
import datetime
import torch
from typing import Dict, Any

# 
def parse_asset_config(config_dict, config_key):
    if config_key not in config_dict:
        print(f"Key '{config_key}' not found in assets dictionary")
        return None, None
    
    entity_config = config_dict[config_key]

    if hasattr(entity_config, '__iter__') and isinstance(entity_config, list):
        if len(entity_config) == 1:
            return entity_config[0], None
        elif len(entity_config) == 2:
            return entity_config[0], entity_config[1]
        else:
            print(f"Value for key '{config_key}' has {len(entity_config)} elements, expected 1 or 2")
            return None, None
    else:
        return entity_config, None

#
def convert_dict_to_tensors(nested_dict: Dict[str, Any], 
                            dtype: torch.dtype = torch.float32,
                            device: str = None) -> Dict[str, Any]:
    """
    将嵌套字典中的所有列表转换为PyTorch张量
    
    Args:
        nested_dict: 包含列表的嵌套字典
        device: 目标设备（如'cpu', 'cuda:0'），如果为None则使用默认设备
        dtype: 目标数据类型，默认为torch.float32
    
    Returns:
        转换后的字典，其中所有列表都变为PyTorch张量
    """
    if nested_dict is None:
        return None
    
    result = {}
    
    for key, value in nested_dict.items():
        if isinstance(value, dict):
            # 递归处理嵌套字典
            result[key] = convert_dict_to_tensors(value, dtype, device)
        elif isinstance(value, (list, tuple)):
            # 转换列表或元组为PyTorch张量
            tensor = torch.tensor(value, dtype=dtype)
            if device is not None:
                tensor = tensor.to(device)
            result[key] = tensor
        elif isinstance(value, torch.Tensor):
            # 如果已经是张量，确保设备和数据类型正确
            tensor = value
            if dtype is not None and tensor.dtype != dtype:
                tensor = tensor.to(dtype)
            if device is not None and tensor.device != torch.device(device):
                tensor = tensor.to(device)
            result[key] = tensor
        else:
            # 保持其他类型不变（数字、字符串等）
            result[key] = value
    
    return result

#
def generate_filename(prefix="video", extension="mp4", folder_path=""):
    """
    生成带时间戳的文件名
    Args:
        prefix (str): 文件名前缀，默认为"video"
        extension (str): 文件扩展名，默认为"mp4"
        folder_path (str): 文件夹路径，如果提供会检查并创建目录
    Returns:
        str: 完整的文件路径
    """
    # 获取当前时间
    now = datetime.datetime.now()
    
    # 格式化时间字符串
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    
    # 组合文件名
    filename = f"{prefix}_{timestamp}.{extension}"
    
    # 如果提供了文件夹路径，确保目录存在并返回完整路径
    if folder_path:
        # 创建目录（如果不存在）
        os.makedirs(folder_path, exist_ok=True)
        return os.path.join(folder_path, filename)
    
    return filename

#
def angle_difference(target, current):
    """
    使用PyTorch计算角度差值, 处理-π到π的跨越问题
    Args:
        target: 目标角度张量
        current: 当前角度张量
    Returns:
        最短路径的角度差值张量
    """
    diff = target - current
    # Limit the difference in [-π, π]
    if isinstance(diff, torch.Tensor):
        # 使用PyTorch操作确保梯度可计算
        diff = torch.remainder(diff + math.pi, 2 * math.pi) - math.pi
    else:
        # 处理标量
        diff = ((diff + math.pi) % (2 * math.pi)) - math.pi
    return diff

def angular_velocity(current_qpos, prev_qpos, dt=None):
    """
    基于当前位置和上一次位置计算角速度
    Args:
        current_qpos: 当前关节位置张量
        prev_qpos: 上一次关节位置张量  
        dt: 持续时间
    Returns:
        目标角速度张量
    """
    if prev_qpos is None:
        return torch.zeros_like(current_qpos)
    
    if dt is None:
        dt = 0.1
    
    # 计算位置差值，处理角度跨越边界的情况
    angle_diff = angle_difference(current_qpos, prev_qpos)
    
    # 应用折扣系数计算目标角速度
    target_velocity = angle_diff / dt
    
    return target_velocity


#### Camera operation

def with_camera(cam_settings):
    """
    Check whether CAM_SETTINGS contains camera configuration
    Args:
        cam_settings (dict): camera configurations
    Returns:
        bool: return True if containing any camera configuration, otherwise return False
    """
    if not isinstance(cam_settings, dict):
        return False
    
    # Check all keys. Look for keys starting with "camera" (e.g., "camera", "camera0", "camera1")
    camera_keys = [key for key in cam_settings.keys() 
                  if isinstance(key, str) and key.startswith("camera")]
    
    return len(camera_keys) > 0

def calculate_camera_pose(wrist_position, wrist_quaternion):
    """
    使用pos、lookat和up参数设置手腕相机位姿
    
    参数:
        wrist_position: torch.Tensor, 手腕位置 [x, y, z]
        wrist_quaternion: torch.Tensor, 手腕四元数 [qw, qx, qy, qz]
    """    
    # 计算相机的朝向方向（假设相机朝向手腕的z轴负方向）
    transform_matrix = as_transform_matrix(wrist_position, wrist_quaternion)
    
    # 提取坐标轴方向
    # 旋转矩阵的列向量分别是x, y, z轴方向
    x_axis = transform_matrix[:3, 0]  # 右方向
    y_axis = transform_matrix[:3, 1]  # 上方向  
    z_axis = transform_matrix[:3, 2]  # 前方向
    
    # 设置相机参数
    pos = wrist_position - z_axis * 0.12 + x_axis * 0.07 # 相机位置就是手腕位置
    
    # 看向点：沿着相机的前方向（z轴负方向是相机的观察方向）
    # 在计算机图形学中，相机通常看向z轴负方向
    lookat = wrist_position + z_axis * 1.0
    
    # 上方向：使用手腕的y轴方向
    up = x_axis

    return pos, lookat, up


### Attitude-related state variables

def reorder_quaternion(quat: torch.Tensor, to_format: str = "xyzw") -> torch.Tensor:
    """
    Convert quaternion between scalar-first [w, x, y, z] and scalar-last [x, y, z, w] formats.
    Args:
        quat: Input quaternion tensor
        to_format: Target format - "xyzw" (x, y, z, w) or "wxyz" (w, x, y, z)
    Returns:
        Quaternion in the specified format
    Raises:
        ValueError: If quaternion doesn't have 4 elements or invalid format specified
    """
    if quat.shape[-1] != 4:
        raise ValueError(f"Quaternion must have 4 elements, got {quat.shape[-1]}")
    
    if to_format not in ["xyzw", "qxqyqzqw", "wxyz", "qwqxqyqz"]:
        raise ValueError(f"Target format must be 'xyzw' or 'wxyz', got '{to_format}'")
    
    if to_format in ["xyzw", "qxqyqzqw"]:
        # Rearrange from [w, x, y, z] to [x, y, z, w]
        return torch.roll(quat, shifts=-1, dims=-1)
    elif to_format in ["wxyz", "qwqxqyqz"]:
        # Rearrange from [x, y, z, w] to [w, x, y, z]
        return torch.roll(quat, shifts=1, dims=-1)
    else:
        raise ValueError(f"Target format must be 'xyzw' or 'wxyz', got '{to_format}'")

def as_rotation_matrix(quaternion, order="wxyz"):
    """
    根据姿态四元数，计算旋转矩阵
    参数:
        quaternion: torch.Tensor, 末端姿态四元数 
        order: string, 定义四元数的元素顺序
    返回:
        rotation_matrix: 旋转矩阵
    """
    # 提取四元数分量
    if order in ["wxyz", "qwqxqyqz"]:
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    elif order in ["xyzw", "qxqyqzqw"]:
        qx, qy, qz, qw = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    else:
        raise ValueError(f"不支持的顺序格式: {order}。支持的格式: 'wxyz', 'xyzw', 'qwqxqyqz', 'qxqyqzqw'")
    
    # 计算旋转矩阵（从四元数到旋转矩阵的转换）
    # 第一列 (x轴方向)
    r00 = 1 - 2*(qy*qy + qz*qz)
    r10 = 2*(qx*qy + qw*qz)
    r20 = 2*(qx*qz - qw*qy)
    
    # 第二列 (y轴方向)
    r01 = 2*(qx*qy - qw*qz)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r21 = 2*(qy*qz + qw*qx)
    
    # 第三列 (z轴方向)
    r02 = 2*(qx*qz + qw*qy)
    r12 = 2*(qy*qz - qw*qx)
    r22 = 1 - 2*(qx*qx + qy*qy)
    
    # 构建旋转矩阵
    rotation_matrix = torch.tensor([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ], device=quaternion.device, dtype=quaternion.dtype)
    
    return rotation_matrix

def as_transform_matrix(position, quaternion, order="wxyz"):
    """
    将位置和四元数转换为4x4平移旋转变换矩阵
    
    参数:
        position: torch.Tensor, 位置 [x, y, z]
        quaternion: torch.Tensor, 四元数 
        order: string, 四元数的元素顺序 (默认"wxyz")
        
    返回:
        torch.Tensor: 4x4变换矩阵
    """
    device = position.device
    dtype = position.dtype
    
    # 使用as_rotation_matrix函数获取3x3旋转矩阵
    rotation_matrix = as_rotation_matrix(quaternion, order=order)
    
    # 构建4x4变换矩阵
    transform_matrix = torch.eye(4, device=device, dtype=dtype)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = position
    
    return transform_matrix

def quaternion_conjugate(q, order="xyzw"):
    """四元数共轭 [qx, qy, qz, qw] -> [-qx, -qy, -qz, qw]"""
    if order in ["xyzw", "qxqyqzqw"]:
        return torch.tensor([-q[0], -q[1], -q[2], q[3]], 
                        dtype=q.dtype, device=q.device)
    elif order in ["wxyzw", "qwqxqyqz"]:
        return torch.tensor([q[0], -q[1], -q[2], -q[3]], 
                        dtype=q.dtype, device=q.device)

def quaternion_multiply(q1, q2):
    """四元数乘法, [qx, qy, qz, qw]"""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    
    return torch.stack([x, y, z, w])


import torch

class SmoothIKSolver:
    def __init__(self, robot, end_effector, smooth_factor=0.3, max_joint_change=0.05, 
                 pos_tolerance=1e-4, quat_tolerance=1e-4):
        self.robot = robot
        self.end_effector = end_effector
        self.smooth_factor = smooth_factor  # 平滑系数
        self.max_joint_change = max_joint_change  # 最大关节变化
        self.pos_tolerance = pos_tolerance  # 位置变化容差
        self.quat_tolerance = quat_tolerance  # 姿态变化容差
        
        # 初始化状态变量
        self.target_pos = None
        self.target_quat = None
        self.last_target_pos = None
        self.last_target_quat = None
        self.last_qpos = None
        
        # 使用字典缓存IK解，但需要注意PyTorch张量不能直接作为键
        self.cache = {}
        
    def _get_cache_key(self, pos, quat):
        """生成缓存键，将张量转换为元组"""
        pos_key = tuple(pos.cpu().numpy().flatten()) if pos is not None else None
        quat_key = tuple(quat.cpu().numpy().flatten()) if quat is not None else None
        return (pos_key, quat_key)
    
    def compute_ik(self, pos, quat):
        """计算逆运动学解，使用缓存提高效率"""
        cache_key = self._get_cache_key(pos, quat)
        
        # 检查是否有缓存结果
        if cache_key in self.cache:
            return self.cache[cache_key].clone().to(pos.device)
        
        # 计算新的IK解
        qpos = self.robot.inverse_kinematics(
            link=self.end_effector,
            pos=pos, 
            quat=quat,
        )
        
        # 缓存结果
        self.cache[cache_key] = qpos.clone().cpu()
        return qpos
    
    def quat_distance(self, q1, q2):
        """计算两个四元数之间的距离（考虑四元数的双覆盖性质）"""
        # 确保四元数是单位四元数
        q1_norm = q1 / torch.norm(q1)
        q2_norm = q2 / torch.norm(q2)
        
        # 计算点积并确保在[-1, 1]范围内
        dot = torch.clamp(torch.dot(q1_norm, q2_norm), -1.0, 1.0)
        
        # 考虑四元数的双覆盖性质（q和-q表示相同的旋转）
        dot = torch.abs(dot)
        
        # 计算角度距离
        return 2 * torch.acos(dot)
    
    def solve(self, pos=None, quat=None):
        """更新目标并计算平滑的逆运动学解"""
        # 检查目标是否变化
        pos_unchanged = (pos is None or 
                        (self.target_pos is not None and 
                         torch.norm(pos - self.target_pos) < self.pos_tolerance))
        
        quat_unchanged = (quat is None or 
                         (self.target_quat is not None and 
                          self.quat_distance(quat, self.target_quat) < self.quat_tolerance))
        
        # 如果目标未变化，返回上一次的解
        if pos_unchanged and quat_unchanged and self.last_qpos is not None:
            return self.last_qpos.clone()
        
        # 更新目标
        old_target_pos = self.target_pos
        old_target_quat = self.target_quat
        
        if pos is not None:
            self.target_pos = pos.clone()
        if quat is not None:
            self.target_quat = quat.clone()
        
        # 计算目标变化量
        pos_diff = 0.0
        quat_diff = 0.0
        
        if old_target_pos is not None and self.target_pos is not None:
            pos_diff = torch.norm(self.target_pos - old_target_pos).item()
        
        if old_target_quat is not None and self.target_quat is not None:
            quat_diff = self.quat_distance(self.target_quat, old_target_quat).item()
        
        # 计算IK解
        qpos = self.compute_ik(self.target_pos, self.target_quat)
        
        # 平滑处理
        if self.last_qpos is not None:
            # 如果目标变化较小，应用平滑
            if pos_diff < 0.01 and quat_diff < 0.01:
                qpos = self.smooth_factor * qpos + (1 - self.smooth_factor) * self.last_qpos
            
            # 限制关节角的最大变化
            delta = qpos - self.last_qpos
            delta_norm = torch.norm(delta).item()
            
            if delta_norm > self.max_joint_change:
                qpos = self.last_qpos + delta * (self.max_joint_change / delta_norm)
        
        # 更新历史状态
        self.last_target_pos = self.target_pos.clone() if self.target_pos is not None else None
        self.last_target_quat = self.target_quat.clone() if self.target_quat is not None else None
        self.last_qpos = qpos.clone()
        
        return qpos
    
    def clear_cache(self):
        """清空缓存"""
        self.cache = {}

# 使用示例
if __name__ == "__main__":
    # 初始化控制器
    IKsolver = SmoothIKSolver(
        robot=robot, 
        end_effector=end_effector,
        smooth_factor=0.3, 
        max_joint_change=0.05,
        pos_tolerance=1e-4,
        quat_tolerance=1e-4
    )

    # 在主循环中
    while True:
        # 获取新的目标位置和姿态
        target_pos = get_target_position()  # 返回torch.Tensor
        target_quat = get_target_orientation()  # 返回torch.Tensor
        
        # 计算平滑的关节角
        qpos = IKsolver.solve(target_pos, target_quat)
        
        # 应用关节角到机器人
        robot.set_qpos(qpos)
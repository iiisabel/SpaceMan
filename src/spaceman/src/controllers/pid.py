import math
import numpy as np
import torch
import sys

from pathlib import Path
current_file_path = Path(__file__).resolve().parent
sys.path.append(str(current_file_path.parent))
from envs.genesis_env import GenesisSim

class PIDController:
    def __init__(self, P, I, D, setpoint, dt=0.01, output_limits=None):
        """
        PyTorch GPU版本的6自由度PID控制器
        
        Args:
            P: 比例增益 [6]
            I: 积分增益 [6]
            D: 微分增益 [6]
            setpoint: 目标值 [6] [x, y, z, roll, pitch, yaw]
            dt: 时间步长
            device: 'cuda' or 'cpu'
        """
        self.device = GenesisSim().device
        self.datatype = GenesisSim().datatype
        self.dt = dt

        self.Kp = torch.tensor(P, device=self.device, dtype=self.datatype)
        self.Ki = torch.tensor(I, device=self.device, dtype=self.datatype)
        self.Kd = torch.tensor(D, device=self.device, dtype=self.datatype)
        self.setpoint = torch.tensor(setpoint, device=self.device, dtype=self.datatype)
        self.output_limits = self._parse_output_limits(output_limits)
        
        self.prev_error = torch.zeros(6, device=self.device, dtype=self.datatype)
        self.integral = torch.zeros(6, device=self.device, dtype=self.datatype)

        self.init_P = self.Kp.clone()
        self.init_I = self.Ki.clone()
        self.init_D = self.Kd.clone()
        self.init_setpoint = self.setpoint.clone()

    def _parse_output_limits(self, output_limits):
        if output_limits is None:
            return None
        elif isinstance(output_limits, (int, float)):
            return [(float(-output_limits), float(output_limits))] * 6
        elif isinstance(output_limits, (list, tuple)):
            if len(output_limits) == 6:
                limits = []
                for limit in output_limits:
                    if limit is None:
                        limits.append((-float('inf'), float('inf')))
                    elif isinstance(limit, (int, float)):
                        limits.append((float(-limit), float(limit)))
                    elif isinstance(limit, (list, tuple)) and len(limit) == 2:
                        limits.append((float(limit[0]), float(limit[1])))
                return limits

    def _wrap_error(self, error, setpoint, current):
        wrapped_error = error.clone()
        
        for i in range(3, 6):
            angle_diff = setpoint[i] - current[i]
            wrapped_error[i] = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        return wrapped_error
    
    def _apply_output_limits(self, output):
        if self.output_limits is None:
            return output
        
        limited_output = output.clone()
        for i in range(6):
            min_val, max_val = self.output_limits[i]
            limited_output[i] = torch.clamp(output[i], min_val, max_val)
        
        return limited_output

    def control(self, pos, orien):
        """
        计算控制输出
        
        Args:
            pos: 当前位置 [x, y, z]
            orien: 当前姿态 [roll, pitch, yaw]
            
        Returns:
            pos_control: 位置控制输出 [x, y, z]
            orien_control: 姿态控制输出 [roll, pitch, yaw]
        """
        if not isinstance(pos, torch.Tensor):
            pos = torch.tensor(pos, device=self.device, dtype=self.datatype)
        else:
            pos = pos.to(self.device)
            
        if not isinstance(orien, torch.Tensor):
            orien = torch.tensor(orien, device=self.device, dtype=self.datatype)
        else:
            orien = orien.to(self.device)
        
        current_state = torch.cat([pos, orien])        
        
        raw_error = self.setpoint - current_state
        error = self._wrap_error(raw_error, self.setpoint, current_state)

        self.integral = self.integral + error * self.dt

        P = self.Kp * error
        I = self.Ki * self.integral
        error_diff = error - self.prev_error

        for i in range(3, 6):
            if torch.abs(error_diff[i]) > math.pi:
                if error_diff[i] > 0:
                    error_diff[i] = error_diff[i] - 2 * math.pi
                else:
                    error_diff[i] = error_diff[i] + 2 * math.pi
        
        D = self.Kd * error_diff / self.dt
        
        self.prev_error = error.clone()
        output = P + I + D

        output = self._apply_output_limits(output)

        pos_control = output[:3]
        orien_control = output[3:]
        
        return pos_control, orien_control
    
    def reset(self):
        self.prev_error.zero_()
        self.integral.zero_()
        self.Kp = self.init_P.clone()
        self.Ki = self.init_I.clone()
        self.Kd = self.init_D.clone()
        self.setpoint = self.init_setpoint.clone()

    def _wrap_to_pi(self, angles):
        wrapped = (angles + math.pi) % (2 * math.pi) - math.pi
        return wrapped
    
    def update_setpoint(self, setpoint):
        self.setpoint = torch.tensor(setpoint, device=self.device, dtype=self.datatype)
        for i in range(3, 6):
            if self.setpoint[i].abs() > math.pi:
                self.setpoint[i] = self._wrap_to_pi(self.setpoint[i])
    
    def update_gains(self, P=None, I=None, D=None):
        if P is not None:
            self.Kp = torch.tensor(P, device=self.device, dtype=self.datatype)
        if I is not None:
            self.Ki = torch.tensor(I, device=self.device, dtype=self.datatype)
        if D is not None:
            self.Kd = torch.tensor(D, device=self.device, dtype=self.datatype)

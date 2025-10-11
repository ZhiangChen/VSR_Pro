# trajectory_generator.py

import math
import time
from typing import List, Tuple, Generator

class TrajectoryGenerator:
    def __init__(
        self,
        duration: float = 20.0,
        timestep: float = 0.01,
        
        amp_x: float = 0.5,
        amp_y: float = 0.5,
        amp_z: float = 0.5,
        amp_roll: float = 0.1,
        amp_pitch: float = 0.1,
        amp_yaw: float = 0.1,

        freq_x: float = 0.5,
        freq_y: float = 0.5,
        freq_z: float = 0.5,
        freq_roll: float = 0.5,
        freq_pitch: float = 0.5,
        freq_yaw: float = 0.5,
    ):
        """
        A trajectory generator that matches your old sine/cos pattern, 
        but with separate amplitude/frequency for each DOF.
        """
        self.duration = duration
        self.timestep = timestep

        self.amp_x = amp_x
        self.amp_y = amp_y
        self.amp_z = amp_z
        self.amp_roll = amp_roll
        self.amp_pitch = amp_pitch
        self.amp_yaw = amp_yaw

        self.freq_x = freq_x
        self.freq_y = freq_y
        self.freq_z = freq_z
        self.freq_roll = freq_roll
        self.freq_pitch = freq_pitch
        self.freq_yaw = freq_yaw

    def generate_trajectory_offline(self) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        num_steps = int(self.duration / self.timestep)
        timestamps = []
        joint_positions = []
        joint_velocities = []

        for i in range(num_steps):
            t = i * self.timestep
            timestamps.append(t)

            # ------- Positions (matching old code’s pattern) -------
            # X => sin
            pos_x = self.amp_x * math.sin(2.0 * math.pi * self.freq_x * t)
            # Y => cos
            pos_y = self.amp_y * math.cos(2.0 * math.pi * self.freq_y * t)
            # Z => sin
            pos_z = self.amp_z * math.sin(2.0 * math.pi * self.freq_z * t)

            # Roll => sin
            roll  = self.amp_roll  * math.sin(2.0 * math.pi * self.freq_roll  * t)
            # Pitch => cos
            pitch = self.amp_pitch * math.cos(2.0 * math.pi * self.freq_pitch * t)
            # Yaw => sin(...) with half frequency => freq_yaw / 2
            yaw   = self.amp_yaw   * math.sin(2.0 * math.pi * (self.freq_yaw / 2.0) * t)

            # ------- Velocities (the derivative) -------
            # d/dt of sin(...) = cos(...)*factor, d/dt of cos(...) = -sin(...)*factor
            vel_x = 2.0 * math.pi * self.freq_x * self.amp_x * math.cos(2.0 * math.pi * self.freq_x * t)
            vel_y = -2.0 * math.pi * self.freq_y * self.amp_y * math.sin(2.0 * math.pi * self.freq_y * t)
            vel_z = 2.0 * math.pi * self.freq_z * self.amp_z * math.cos(2.0 * math.pi * self.freq_z * t)

            vroll  = 2.0 * math.pi * self.freq_roll  * self.amp_roll  * math.cos(2.0 * math.pi * self.freq_roll  * t)
            vpitch = -2.0 * math.pi * self.freq_pitch * self.amp_pitch * math.sin(2.0 * math.pi * self.freq_pitch * t)
            # yaw derivative, freq => freq_yaw/2 => multiply by (freq_yaw/2)
            v_yaw_factor = 2.0 * math.pi * (self.freq_yaw / 2.0)
            vyaw   = v_yaw_factor * self.amp_yaw * math.cos(v_yaw_factor * t)

            joint_positions.append([pos_x, pos_y, pos_z, roll, pitch, yaw])
            joint_velocities.append([vel_x, vel_y, vel_z, vroll, vpitch, vyaw])

        return joint_positions, joint_velocities, timestamps

    def generate_trajectory_realtime(self) -> Generator[Tuple[List[float], List[float], float], None, None]:
        num_steps = int(self.duration / self.timestep)
        for i in range(num_steps):
            t = i * self.timestep

            # Positions
            pos_x = self.amp_x * math.sin(2.0 * math.pi * self.freq_x * t)
            pos_y = self.amp_y * math.cos(2.0 * math.pi * self.freq_y * t)
            pos_z = self.amp_z * math.sin(2.0 * math.pi * self.freq_z * t)

            roll  = self.amp_roll  * math.sin(2.0 * math.pi * self.freq_roll  * t)
            pitch = self.amp_pitch * math.cos(2.0 * math.pi * self.freq_pitch * t)
            yaw   = self.amp_yaw   * math.sin(2.0 * math.pi * (self.freq_yaw / 2.0) * t)

            # Velocities
            vel_x = 2.0 * math.pi * self.freq_x * self.amp_x * math.cos(2.0 * math.pi * self.freq_x * t)
            vel_y = -2.0 * math.pi * self.freq_y * self.amp_y * math.sin(2.0 * math.pi * self.freq_y * t)
            vel_z = 2.0 * math.pi * self.freq_z * self.amp_z * math.cos(2.0 * math.pi * self.freq_z * t)

            vroll  = 2.0 * math.pi * self.freq_roll  * self.amp_roll  * math.cos(2.0 * math.pi * self.freq_roll  * t)
            vpitch = -2.0 * math.pi * self.freq_pitch * self.amp_pitch * math.sin(2.0 * math.pi * self.freq_pitch * t)
            v_yaw_factor = 2.0 * math.pi * (self.freq_yaw / 2.0)
            vyaw   = v_yaw_factor * self.amp_yaw * math.cos(v_yaw_factor * t)

            yield ([pos_x, pos_y, pos_z, roll, pitch, yaw],
                   [vel_x, vel_y, vel_z, vroll, vpitch, vyaw],
                   t)

            time.sleep(self.timestep)

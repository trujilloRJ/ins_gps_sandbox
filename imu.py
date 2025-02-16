import numpy as np
from scipy.spatial.transform import Rotation
from utils import euler_to_quaternion, quaternion_to_euler


class IMUNavigator():
    def __init__(self, p, v, roll, pitch, yaw):
        self.pos = p   # position vector
        self.vel = v   # velocity vector
        self.qatt = euler_to_quaternion(roll, pitch, yaw)  # attitude as quaternion

    def __repr__(self):
        # Provide a concise summary of the current state
        pos_str = f"Position: {self.pos}"
        vel_str = f"Velocity: {self.vel}"
        roll, pitch, yaw = quaternion_to_euler(self.qatt)
        att_str = f"Attitude (Euler angles): Roll={np.degrees(roll):.2f}°, Pitch={np.degrees(pitch):.2f}°, Yaw={np.degrees(yaw):.2f}°"

        return f"IMUNavigator(\n\t{pos_str}\n\t{vel_str}\n\t{att_str})"

    def step(self, acc, gyro, dt):
        g = np.array([0, 0, 9.81])  

        # Convert quaternion to rotation matrix (body to navigation)
        base_rot = Rotation.from_quat(self.qatt)
        Rbn = base_rot.as_matrix()

        # Transform acceleration from body frame to navigation frame
        acc_nav = Rbn @ acc - g

        # Translational prediction
        self.pos = self.pos + self.vel * dt + 0.5 * acc_nav * dt**2
        self.vel = self.vel + acc_nav * dt

        # Convert angular velocity to quaternion derivative
        omega = np.hstack(([0], gyro))  # Convert to quaternion form
        q_dot = 0.5 * base_rot.as_quat() @ omega
        self.qatt = self.qatt + q_dot * dt
        self.qatt /= np.linalg.norm(self.qatt)
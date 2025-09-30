import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist

import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

class EKFFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Odometry, '/fused_odom', 10)

        # State: [x, y, theta]
        self.state = np.zeros((3, 1))  
        self.P = np.eye(3) * 0.1  # Covariance

        self.last_time = self.get_clock().now()

        # For plotting
        self.odom_positions = []
        self.fused_positions = []

        # Start plotting in a separate thread
        Thread(target=self.plot_loop, daemon=True).start()

    def odom_callback(self, msg: Odometry):
        # Time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Odometry velocity inputs
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Predict step (motion model)
        theta = self.state[2, 0]
        F = np.eye(3)
        B = np.array([
            [np.cos(theta)*dt, 0],
            [np.sin(theta)*dt, 0],
            [0, dt]
        ])
        u = np.array([[v], [w]])
        self.state = self.state + B @ u

        # Update covariance
        Q = np.diag([0.01, 0.01, 0.01])
        self.P = F @ self.P @ F.T + Q

        # Save raw odometry for plotting
        self.odom_positions.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def imu_callback(self, msg: Imu):
        # Measurement model: orientation yaw
        q = msg.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        measured_theta = np.arctan2(siny_cosp, cosy_cosp)

        # Measurement update (correct step)
        z = np.array([[measured_theta]])
        H = np.array([[0, 0, 1]])  # we only observe theta
        R = np.array([[0.05]])    # IMU measurement noise

        y = z - H @ self.state  # innovation
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        # Publish fused odometry
        self.publish_fused_odom()

        # Save fused position for plotting
        self.fused_positions.append((self.state[0, 0], self.state[1, 0]))

    def publish_fused_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = float(self.state[0])
        msg.pose.pose.position.y = float(self.state[1])
        msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        theta = float(self.state[2])
        qz = np.sin(theta / 2.0)
        qw = np.cos(theta / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.pub.publish(msg)

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        while True:
            ax.clear()
            if self.odom_positions:
                ox, oy = zip(*self.odom_positions)
                ax.plot(ox, oy, label='Raw Odometry', color='red')
            if self.fused_positions:
                fx, fy = zip(*self.fused_positions)
                ax.plot(fx, fy, label='Fused EKF', color='green')
            ax.set_title("Robot Position (EKF vs Raw Odometry)")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.legend()
            plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        
        # Setup plot
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Live Odometry Comparison')
        self.ax.grid(True)
        
        # Initialize empty plots
        self.raw_line, = self.ax.plot([], [], 'r-', label='Raw Odometry', alpha=0.7)
        self.filtered_line, = self.ax.plot([], [], 'b-', label='Filtered Odometry', linewidth=1.5)
        self.ax.legend()
        
        # Data storage
        self.raw_data = {'x': [], 'y': []}
        self.filtered_data = {'x': [], 'y': []}
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.raw_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.filtered_callback, 10)
        
        self.get_logger().info("Live odometry plotter ready. Move the robot to see data!")

    def raw_callback(self, msg):
        self.update_data(msg, is_raw=True)

    def filtered_callback(self, msg):
        self.update_data(msg, is_raw=False)

    def update_data(self, msg, is_raw):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if is_raw:
            self.raw_data['x'].append(x)
            self.raw_data['y'].append(y)
            self.raw_line.set_data(self.raw_data['x'], self.raw_data['y'])
        else:
            self.filtered_data['x'].append(x)
            self.filtered_data['y'].append(y)
            self.filtered_line.set_data(self.filtered_data['x'], self.filtered_data['y'])
        
        # Auto-scale and redraw
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
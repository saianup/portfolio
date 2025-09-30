import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import threading
from collections import deque
import time

class ImuPlotter(Node):
    def __init__(self):
        super().__init__('imu_plotter')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.acc_x = deque(maxlen=1000)  # Will store IMU Y data
        self.acc_y = deque(maxlen=1000)  # Will store IMU Z data
        self.timestamps = deque(maxlen=1000)

        self.lock = threading.Lock()
        self.start_time = time.time()
        self.running = True

    def imu_callback(self, msg):
        with self.lock:
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            self.acc_x.append(msg.linear_acceleration.y)  # Y as X
            self.acc_y.append(msg.linear_acceleration.z)  # Z as Y

def plot_imu_data(node):
    plt.ion()
    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label='Acceleration X (from Y)', color='blue')
    line_y, = ax.plot([], [], label='Acceleration Y (from Z)', color='red')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('IMU Acceleration (Tilt-Adjusted)')
    ax.legend()
    ax.grid(True)

    while rclpy.ok() and node.running:
        with node.lock:
            timestamps = list(node.timestamps)
            acc_x_vals = list(node.acc_x)
            acc_y_vals = list(node.acc_y)

        if timestamps:
            line_x.set_data(timestamps, acc_x_vals)
            line_y.set_data(timestamps, acc_y_vals)

            ax.relim()
            ax.autoscale_view()

            plt.draw()
            plt.pause(0.1)

    plt.ioff()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPlotter()

    plot_thread = threading.Thread(target=plot_imu_data, args=(node,))
    plot_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Plotting terminated by user.')
    finally:
        node.running = False
        plot_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

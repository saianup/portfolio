import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import threading

class LiveEncoderPlot(Node):
    def __init__(self):
        super().__init__('live_encoder_plotter')
        self.subscription = self.create_subscription(
            Point,
            '/encoder_pos',
            self.listener_callback,
            10)
        self.x_vals = []
        self.y_vals = []
        self.lock = threading.Lock()
        self.running = True

    def listener_callback(self, msg):
        with self.lock:
            self.x_vals.append(msg.x * 100)  # convert to cm
            self.y_vals.append(msg.y * 100)

def plot_live(node):
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-')
    ax.set_xlabel('X Position (cm)')
    ax.set_ylabel('Y Position (cm)')
    ax.set_title('Live Robot Pose from /encoder_pos')
    ax.grid(True)

    while rclpy.ok() and node.running:
        with node.lock:
            x_data = node.x_vals.copy()
            y_data = node.y_vals.copy()

        if x_data and y_data:
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.1)

    plt.ioff()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = LiveEncoderPlot()
    
    plot_thread = threading.Thread(target=plot_live, args=(node,))
    plot_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Plotting stopped by user.")
    finally:
        node.running = False
        plot_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

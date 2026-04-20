#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections
import threading
import time

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        
        # Track joint_2 by default, but it can be changed via parameters
        self.declare_parameter('target_joint', 'joint_2')
        self.target_joint = self.get_parameter('target_joint').value
        
        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(JointState, '/faulty_joint_states', self.faulty_cb, 10)
        self.create_subscription(Float64MultiArray, '/ekf/residuals', self.residual_cb, 10)
        
        # Deques for the "Scrolling" effect. 
        # Keeping maxlen=500 points (approx 10 seconds of data at 50Hz plotting rate)
        maxlen = 500
        self.times = collections.deque(maxlen=maxlen)
        self.true_pos = collections.deque(maxlen=maxlen)
        self.faulty_pos = collections.deque(maxlen=maxlen)
        self.res_pos = collections.deque(maxlen=maxlen)
        
        self.start_time = None
        self.last_plot_time = 0

        # For indexing residuals, we assume joint order from ekf_node: joint_1..5
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        try:
            self.res_index = self.joint_names.index(self.target_joint)
        except ValueError:
            self.res_index = 1 # default to joint_2
            
        # State variables
        self.latest_true = 0.0
        self.latest_faulty = 0.0
        self.latest_res = 0.0
        
        self.get_logger().info(f"Live Plotter started. Tracking '{self.target_joint}' residuals and inputs.")

    def get_time(self):
        if self.start_time is None:
            self.start_time = time.time()
        return time.time() - self.start_time

    def joint_cb(self, msg):
        try:
            idx = msg.name.index(self.target_joint)
            self.latest_true = msg.position[idx]
            self._update_data()
        except ValueError:
            pass

    def faulty_cb(self, msg):
        try:
            idx = msg.name.index(self.target_joint)
            self.latest_faulty = msg.position[idx]
        except ValueError:
            pass

    def residual_cb(self, msg):
        if len(msg.data) >= 5:
            # First 5 dimensions are positions
            self.latest_res = msg.data[self.res_index]

    def _update_data(self):
        # Throttle data processing to ~50 Hz so we don't freeze the GUI with 1000Hz Gazebo data
        t = time.time()
        if t - self.last_plot_time > 0.02:
            self.times.append(self.get_time())
            self.true_pos.append(self.latest_true)
            self.faulty_pos.append(self.latest_faulty)
            self.res_pos.append(self.latest_res)
            self.last_plot_time = t

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()
    
    # Spin ROS 2 in a background daemon thread so it doesn't block the UI
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    
    # Set up Matplotlib Figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    fig.canvas.manager.set_window_title('Live Fault Detection Plotter')
    
    # Intialize empty lines
    line_true, = ax1.plot([], [], label='True Position', color='blue', lw=2)
    line_faulty, = ax1.plot([], [], label='Faulted Position', color='red', linestyle='--', lw=2)
    ax1.set_title(f'Inputs: {node.target_joint} Position')
    ax1.set_ylabel('Radians')
    ax1.legend(loc='upper right')
    ax1.grid(True)
    
    line_res, = ax2.plot([], [], label='EKF Position Residual', color='orange', lw=2)
    ax2.set_title(f'Output: EKF Residual ({node.target_joint})')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (rad)')
    ax2.legend(loc='upper right')
    ax2.grid(True)
    
    plt.tight_layout()

    # Animation update loop
    def update_plot(frame):
        if len(node.times) > 0:
            times = list(node.times)
            
            line_true.set_data(times, list(node.true_pos))
            line_faulty.set_data(times, list(node.faulty_pos))
            line_res.set_data(times, list(node.res_pos))
            
            # Dynamic X-Axis (Creates the SCROLLING effect)
            x_max = times[-1]
            x_min = max(0, x_max - 10.0) # Show exactly the last 10 seconds
            
            for ax in [ax1, ax2]:
                ax.set_xlim(x_min, max(10.0, x_max))
                
            # Dynamic Y-Axis for the input wave
            if len(node.true_pos) > 0 and len(node.faulty_pos) > 0:
                y_min1 = min(min(node.true_pos), min(node.faulty_pos))
                y_max1 = max(max(node.true_pos), max(node.faulty_pos))
                ax1.set_ylim(y_min1 - 0.05, y_max1 + 0.05)
            
            # The residual is typically around 0.0, but faults can spike it
            ax2.set_ylim(-0.05, 0.05)
            
        return line_true, line_faulty, line_res

    # Run animation loop at ~20 FPS (50ms interval)
    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    
    # Cleanup upon closing the plot window
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
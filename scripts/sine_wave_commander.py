#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import math
import time
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SineWaveCommander(Node):
    def __init__(self):
        super().__init__('sine_wave_commander')
        
        # Publisher to the arm_controller's command topic
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
        # 100 Hz publishing rate matches our controller update rate
        timer_period = 0.01  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.start_time = time.time()
        
        # Sine wave parameters
        self.amplitude = 0.2  # radians, reduced to avoid self-collisions
        self.frequency = 0.5  # Hz
        
        # Noise parameters
        self.declare_parameter('enable_noise', False)
        self.declare_parameter('noise_std_dev', 0.02)
        
        self.get_logger().info('Sine Wave Commander Node started, commanding all joints...')

    def timer_callback(self):
        msg = Float64MultiArray()
        
        # Time since the node started
        current_time = time.time() - self.start_time
        
        # Get noise parameters
        enable_noise = self.get_parameter('enable_noise').get_parameter_value().bool_value
        noise_std_dev = self.get_parameter('noise_std_dev').get_parameter_value().double_value
        
        # The arm_controller expects 5 values, one for each joint respectively 
        # (joint_1, joint_2, joint_3, joint_4, joint_5)
        # Apply a sine wave to all joints. We use a slight phase shift (i * 0.5) 
        # for each joint so the arm moves in a fluid workspace.
        val = []
        for i in range(5):
            base_val = self.amplitude * math.sin(2.0 * math.pi * self.frequency * current_time + (i * 0.5))
            if enable_noise:
                base_val += random.gauss(0.0, noise_std_dev)
            val.append(base_val)
            
        msg.data = val
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

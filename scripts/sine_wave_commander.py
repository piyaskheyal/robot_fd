#!/usr/bin/env python3
import math
import time

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
        self.amplitude = 1.0  # radians
        self.frequency = 0.5  # Hz
        
        self.get_logger().info('Sine Wave Commander Node started, commanding joint_1...')

    def timer_callback(self):
        msg = Float64MultiArray()
        
        # The arm_controller expects 5 values, one for each joint respectively 
        # (joint_1, joint_2, joint_3, joint_4, joint_5)
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Time since the node started
        current_time = time.time() - self.start_time
        
        # Compute the sine wave explicitly targeting the primary joint (index 0 / joint_1)
        msg.data[0] = self.amplitude * math.sin(2.0 * math.pi * self.frequency * current_time)
        
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

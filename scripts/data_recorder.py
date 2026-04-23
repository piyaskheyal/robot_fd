#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
import ament_index_python.packages

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        
        # Subscribe to EKF residuals
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/ekf/residuals',
            self.listener_callback,
            10)
        
        # Open CSV file for logging
        self.file_name = 'healthy_residuals.csv'
        self.file = open(self.file_name, mode='w', newline='')
        self.csv_writer = csv.writer(self.file)
        
        # Write header: [res_q1...res_q6, res_dq1...res_dq6]
        header = [f'res_q{i}' for i in range(1, 7)] + [f'res_dq{i}' for i in range(1, 7)]
        self.csv_writer.writerow(header)
        
        self.sample_count = 0
        # Optional: stop after acquiring sufficient samples. E.g., 65000 from the paper.
        self.max_samples = 65000
        
        self.get_logger().info(f"Recording residuals to {self.file_name}...")

    def listener_callback(self, msg):
        if self.sample_count < self.max_samples:
            self.csv_writer.writerow(msg.data)
            self.sample_count += 1
            if self.sample_count % 1000 == 0:
                self.get_logger().info(f"Recorded {self.sample_count}/{self.max_samples} samples.")
        elif self.sample_count == self.max_samples:
            self.get_logger().info("Target sample count reached. You can stop this node.")
            self.sample_count += 1  # Increment so we only print this once

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(f"Closed {self.file_name}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
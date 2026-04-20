#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import copy

class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__('fault_injector')

        # Parameters for the fault
        self.declare_parameter('fault_joint_name', 'joint_2')
        self.declare_parameter('fault_magnitude', 0.1) # Add 0.1 rad to position
        
        self.fault_joint = self.get_parameter('fault_joint_name').value
        self.fault_magnitude = self.get_parameter('fault_magnitude').value

        # Subscribers and Publishers
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.create_subscription(Empty, '/inject_fault', self.trigger_cb, 10)
        
        self.faulty_pub = self.create_publisher(JointState, '/faulty_joint_states', 10)
        
        self.fault_active = False

        self.get_logger().info(f"Fault Injector initialized. Target: {self.fault_joint}, Magnitude: {self.fault_magnitude} rad")
        self.get_logger().warn("Waiting for trigger! Run: ros2 topic pub -1 /inject_fault std_msgs/msg/Empty")

    def trigger_cb(self, msg):
        if not self.fault_active:
            self.fault_active = True
            self.get_logger().error(f"INJECTING FAULT NOW! Added {self.fault_magnitude} rad to {self.fault_joint} position.")

    def joint_states_cb(self, msg):
        # Create a deep copy of the message so we don't mutate the original if it's referenced elsewhere
        faulty_msg = copy.deepcopy(msg)

        # Check if it's time to inject the fault
        if self.fault_active:
            try:
                # Find the index of the target joint
                idx = faulty_msg.name.index(self.fault_joint)
                
                # Convert tuple (which ros2 python often uses for message arrays) to list to mutate it
                positions = list(faulty_msg.position)
                
                # Inject the sensor bias (simulate a skipped encoder or miscalibration)
                positions[idx] += self.fault_magnitude
                
                # Assign it back
                faulty_msg.position = tuple(positions)
                
            except ValueError:
                pass # The target joint isn't in this particular JointState message array

        # Always publish the message (it is clean before fault_start_time, and faulty after)
        self.faulty_pub.publish(faulty_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
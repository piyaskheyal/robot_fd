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

        # Dictionary to track multiple active injected faults: {joint_name: magnitude_offset}
        self.active_faults = {}

        # Subscribers and Publishers
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.create_subscription(JointState, '/inject_fault', self.trigger_cb, 10)
        
        self.faulty_pub = self.create_publisher(JointState, '/faulty_joint_states', 10)

        self.get_logger().info(f"Fault Injector dynamically waiting for JointState commands on /inject_fault...")

    def trigger_cb(self, msg):
        # A trigger has arrived containing a joint array and position array
        # Loop through elements and add/update them in our active faults dict
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                magnitude = msg.position[i]
                if magnitude != 0.0:
                    self.active_faults[joint_name] = magnitude
                    self.get_logger().error(f"FAULT INSTRUCTED! Applying {magnitude} rad override to {joint_name}.")
                else: 
                    # Providing 0.0 clears/heals the fault
                    if joint_name in self.active_faults:
                        del self.active_faults[joint_name]
                        self.get_logger().info(f"CLEARED fault on {joint_name}.")

    def joint_states_cb(self, msg):
        # Create a deep copy of the message so we don't mutate the original if it's referenced elsewhere
        faulty_msg = copy.deepcopy(msg)

        # Convert position tuple to list explicitly so we can mutate it
        positions = list(faulty_msg.position)

        # Inject any active tracking sensory biases
        for faulted_joint, magnitude in self.active_faults.items():
            if faulted_joint in faulty_msg.name:
                idx = faulty_msg.name.index(faulted_joint)
                positions[idx] += magnitude

        # Re-pack back into tuple
        faulty_msg.position = tuple(positions)

        # Always publish the message
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
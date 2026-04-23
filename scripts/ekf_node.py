#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import os

import numpy as np
import pinocchio as pin

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # Determine URDF path and load Pinocchio model (Kinematic base)
        pkg_share = get_package_share_directory('robot_arm_description')
        urdf_path = os.path.join(pkg_share, 'model', 'robot_arm_urdf.xacro')
        
        # Note: If xacro needs processing, we might want to subscribe to /robot_description instead,
        # but for simple kinematics, many users just load the plain URDF if it's parsable by Pinocchio.
        # Alternatively, we just use a generic N-DOF kinematic matrix since we only care about the
        # active joints: joint_1 to joint_6.
        
        self.num_joints = 6
        self.last_msg_time = None
        
        # State vector x = [q1..q5, dq1..dq5]^T
        self.x = np.zeros((self.num_joints * 2, 1))
        self.u = np.zeros((self.num_joints, 1)) # The commanded positions
        
        # EKF Covariance Matrices
        self.P = np.eye(self.num_joints * 2) * 1.0 # Initial uncertainty
        
        # INCREASE process noise (Q) because our simplistic model doesn't perfectly match Gazebo dynamics.
        # This prevents the EKF from strictly trusting its imperfect internal model, reducing leakage.
        self.Q = np.eye(self.num_joints * 2) * 0.1 
        
        # Measurement noise (sensor fidelity) - Gazebo is currently giving us perfect data.
        self.R = np.eye(self.num_joints * 2) * 0.01 
        
        # Simple Transition Matrix F based on position controller dynamics:
        # We will update these dynamically in the callback using the real dt.
        # Tuned Kp and Kd to better approximate Gazebo's internal implicit position tracking.
        self.Kp = 150.0  # Increased proportional gain to match Gazebo's stiff position tracking
        self.Kd = 20.0   # Increased derivative damping
        
        self.F = np.eye(self.num_joints * 2)
        self.B = np.zeros((self.num_joints * 2, self.num_joints))

        # Measurement Matrix H (we measure both position and velocity from Gazebo)
        self.H = np.eye(self.num_joints * 2)
        
        # Publishers and Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Float64MultiArray, '/arm_controller/commands', self.command_cb, 10)
        
        self.residual_pub = self.create_publisher(Float64MultiArray, '/ekf/residuals', 10)
        
        self.get_logger().info("EKF Node initialized. Waiting for commands and joint states...")
        self.last_time = self.get_clock().now()

    def command_cb(self, msg):
        # We assume the command matches the 6 joints implicitly
        if len(msg.data) == self.num_joints:
            self.u = np.array(msg.data).reshape(self.num_joints, 1)

    def joint_state_cb(self, msg):
        # Calculate dynamic loop time (dt) instead of hardcoding 0.01
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is None:
            self.last_msg_time = current_time
            return
            
        dt = current_time - self.last_msg_time
        self.last_msg_time = current_time
        
        # Prevent huge jumps if Gazebo starts/stops
        if dt <= 0.0 or dt > 0.1:
            return

        # Update F and B dynamically using the calculated true dt
        self.F[0:6, 6:12] = np.eye(6) * dt
        self.F[6:12, 0:6] = -np.eye(6) * dt * self.Kp
        self.F[6:12, 6:12] = np.eye(6) * (1.0 - dt * self.Kd)

        self.B[6:12, 0:6] = np.eye(6) * dt * self.Kp

        # Filter for our active joints
        target_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        try:
            indices = [msg.name.index(act_j) for act_j in target_joints]
        except ValueError:
            # We don't have all required joints in this message yet
            return
            
        z_q = np.array([msg.position[i] for i in indices]).reshape(6, 1)
        z_dq = np.array([msg.velocity[i] for i in indices]).reshape(6, 1)
        z = np.vstack((z_q, z_dq))
        
        # 1. Predict
        # x_pred = F * x + B * u
        x_pred = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        P_pred = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        
        # 2. Update (Innovation/Residual)
        residual = z - np.dot(self.H, x_pred)
        
        y = residual  # "y" is the innovation in filter terms
        S = np.dot(np.dot(self.H, P_pred), self.H.T) + self.R
        K = np.dot(np.dot(P_pred, self.H.T), np.linalg.inv(S))
        
        self.x = x_pred + np.dot(K, y)
        self.P = P_pred - np.dot(np.dot(K, self.H), P_pred)
        
        # Publish Residuals
        res_msg = Float64MultiArray()
        # Flatten and convert to python list format
        res_msg.data = y.flatten().tolist()
        self.residual_pub.publish(res_msg)
        
        # Optionally log the norm of the position residual for monitoring
        pos_residual_norm = np.linalg.norm(y[0:6])
        # self.get_logger().info(f"Position Residual Norm: {pos_residual_norm:.5f}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
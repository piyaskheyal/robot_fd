#!/home/kheyal/ws_ros2/ws_robot_arm_fault_detection/.venv/bin/python
import os
import collections
import numpy as np
import torch
import torch.nn as nn
import joblib

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

# Re-define Autoencoder architecture exactly as it was during training
class Conv1DAutoencoder(nn.Module):
    def __init__(self, in_channels, seq_len):
        super(Conv1DAutoencoder, self).__init__()
        
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv1d(in_channels=in_channels, out_channels=32, kernel_size=3, padding=1, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2), # seq_len = 25
            
            nn.Conv1d(in_channels=32, out_channels=16, kernel_size=3, padding=1, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=5)  # seq_len = 5
        )
        
        # Decoder
        self.decoder = nn.Sequential(
            nn.ConvTranspose1d(in_channels=16, out_channels=32, kernel_size=5, stride=5), # seq_len = 25
            nn.ReLU(),
            
            nn.ConvTranspose1d(in_channels=32, out_channels=32, kernel_size=2, stride=2), # seq_len = 50
            nn.ReLU(),
            
            nn.Conv1d(in_channels=32, out_channels=in_channels, kernel_size=3, padding=1, stride=1) # seq_len = 50
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded


class AnomalyDetectorNode(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        
        self.declare_parameter('model_dir', 'src/robot_fd/weights')
        model_dir = self.get_parameter('model_dir').value
        
        # Absolute paths based on workspace execution
        # (Alternatively, you could install weights into the `share/` dir, but we read from src for dev)
        self.model_path = os.path.join(model_dir, 'autoencoder_1d_cnn.pth')
        self.scaler_path = os.path.join(model_dir, 'scaler.pkl')

        if not os.path.exists(self.model_path) or not os.path.exists(self.scaler_path):
            self.get_logger().error(f"Cannot find weights/scaler at {model_dir}. Ensuring you trained the model first.")
            return

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Load artifacts
        self.get_logger().info(f"Loading anomaly detector weights from {model_dir} to {self.device}...")
        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        self.threshold = checkpoint['threshold']
        self.window_size = checkpoint['seq_len']
        self.in_channels = checkpoint['in_channels']
        
        self.model = Conv1DAutoencoder(self.in_channels, self.window_size).to(self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        
        self.scaler = joblib.load(self.scaler_path)

        # Buffer to keep the last `window_size` samples (sliding window)
        self.residual_buffer = collections.deque(maxlen=self.window_size)
        
        # Subscriptions & Publishers
        self.create_subscription(Float64MultiArray, '/ekf/residuals', self.residual_cb, 10)
        self.anomaly_pub = self.create_publisher(Bool, '/anomaly_detected', 10)
        
        self.get_logger().info(f"Anomaly Detector initialized. Sliding Window: {self.window_size}, Threshold: {self.threshold:.5f}")

    def residual_cb(self, msg):
        # We expect data to be [res_q1..q5, res_dq1..dq5] (10 dims)
        if len(msg.data) != self.in_channels:
            self.get_logger().warn(f"Expected {self.in_channels} residual features, got {len(msg.data)}")
            return
            
        # Append current reading
        self.residual_buffer.append(list(msg.data))
        
        # Only predict if the window is fully populated
        if len(self.residual_buffer) == self.window_size:
            # 1. Convert to numpy array (Seq_Len, Channels)
            window_np = np.array(self.residual_buffer)
            
            # 2. Scale using the fitted scaler
            window_scaled = self.scaler.transform(window_np)
            
            # 3. Transpose to (Channels, Seq_Len) and add Batch dim -> (1, Channels, Seq_Len)
            window_scaled = window_scaled.T
            window_tensor = torch.tensor(window_scaled, dtype=torch.float32).unsqueeze(0).to(self.device)
            
            # 4. Inference
            with torch.no_grad():
                reconstruction = self.model(window_tensor)
                
                # 5. Calculate MSE Loss matching the training objective
                # (Average over channels and sequence length for this single sequence)
                mse_loss = torch.mean((reconstruction - window_tensor)**2).item()
                
            # 6. Evaluate Anomaly Criterion
            is_anomaly = mse_loss > self.threshold
            
            # Publish True if anomaly, False if nominal
            anomaly_msg = Bool()
            anomaly_msg.data = bool(is_anomaly)
            self.anomaly_pub.publish(anomaly_msg)
            
            # Log warnings at 1 Hz max if an anomaly is detected, or 5 Hz if tracking nominal
            if is_anomaly:
                self.get_logger().warn(f"ANOMALY DETECTED! MSE: {mse_loss:.5f} > THRESH: {self.threshold:.5f}", throttle_duration_sec=0.5)
            else:
                self.get_logger().info(f"System Nominal. MSE: {mse_loss:.5f}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
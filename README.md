# Robot Arm Fault Detection (`robot_fd`)

## Overview
The `robot_fd` package is a complete simulation and machine learning pipeline for performing hybrid model-based and data-driven fault detection on a 5-DOF robotic arm. It operates in **ROS 2 Jazzy** and **Gazebo Harmonic**.

The architecture works by feeding expected control commands and noisy simulated sensor readings into an **Extended Kalman Filter (EKF)**. The EKF calculates structural **residuals** (the mathematical difference between expected state and measured state). These residuals are then analyzed in real-time by a **PyTorch 1D-CNN Autoencoder** to identify unmodeled anomalies, such as dynamic sensor biases or degraded joints.

---

## 🧩 Node Arsenal & Script Descriptions

### Core ROS 2 Nodes
*   **`sine_wave_commander.py`**: 
    Drives the robot through its workspace by publishing `Float64MultiArray` commands (at 100Hz) to the `/arm_controller/commands` topic. It generates a continuous sine wave for all 5 joints with phase offsets to prevent collisions. It features a `0.5` rad DC offset to prevent URDF lower-limit clamping and supports optional Gaussian noise injection.
    *   *Parameters:* `enable_noise` (bool), `noise_std_dev` (double).
*   **`ekf_node.py`**: 
    The mathematical core. It subscribes to `/joint_states` (measurements) and `/arm_controller/commands` (control inputs), executing a 10-dimensional kinematic prediction. It outputs a 10D error metric (5 position residuals + 5 velocity residuals) to the `/ekf/residuals` topic.
*   **`fault_injector.py`**: 
    The adversarial node. It intercepts the pure `/joint_states` from Gazebo and acts as a middleman, publishing corrupted data to `/faulty_joint_states`. It actively listens to the `/inject_fault` topic (`sensor_msgs/msg/JointState`) to dynamically apply, update, or heal biases on any specified joint in real-time.
*   **`anomaly_detector.py`**: 
    The live AI inference engine. It loads the `autoencoder_weights.pth`, `scaler.pkl`, and `threshold.txt`. Subscribing to `/ekf/residuals`, it maintains a sliding window `deque` of the last 50 timesteps. Every 50ms, it runs the PyTorch model, computes the Mean Squared Error (MSE) loss, and flags an `ANOMALY DETECTED!`warningifthelossstrictlyexceedsthedynamicthreshold.
    *   *Parameters:* `model_dir` (Default: `src/robot_fd/weights`).

### Standalone Python Scripts
*   **`data_recorder.py`**: 
    Subscribes to `/ekf/residuals` to capture 65,000 samples (approx. 10 minutes) of healthy, baseline behavior. Saves the output directly to `healthy_residuals.csv`.
*   **`train_autoencoder.py`**: 
    The offline PyTorch training loop. Reads `healthy_residuals.csv`, creates sequences of length 50, scales the data, and trains a 1D Convolutional Neural Network (1D-CNN) Autoencoder. It strictly computes a safe anomaly threshold (`max_healthy_loss * 1.20`) to eliminate momentum-turnaround false positives. It saves the final model bundle unconditionally to `src/robot_fd/weights`.
*   **`live_plotter.py`**: 
    A multi-threaded, real-time Matplotlib visualization tool. It charts the exact offset injected by `fault_injector.py` side-by-side with the explosive response of the EKF residuals using a localized 10-second scrolling window. 
    *   *Parameters:* `target_joint` (string, e.g., `joint_4`).

---

## 🚀 Launch Files

### `fault_detection.launch.py` (Data Collection)
Used strictly to capture uncorrupted, pristine Gazebo data. It launches Gazebo, RViz, the controllers, the EKF, and the sine wave commander.
*   `use_sim_time` *(default: true)*: Synchronizes ROS 2 with Gazebo physics.
*   `enable_noise` *(default: true)*: Applies baseline Gaussian noise to the real system.
*   `noise_std_dev` *(default: 0.035)*: The standard deviation of the baseline noise.

### `test_fault_detection.launch.py` (Orchestrator)
The primary execution orchestrator for testing the neural network. It launches everything in `fault_detection.launch.py` but fundamentally alters the data routing: 
It remaps the `ekf_node` to subscribe to `/faulty_joint_states` instead of `/joint_states`, forcing the EKF to process the corrupted data published by the `fault_injector.py`. It also simultaneously boots the neural net via `anomaly_detector.py`.

---

## 📖 Complete Workflow: From Zero to Fault Detection

### Step 1: Environment Setup
This package strictly relies on a Python 3.12 virtual environment (named `.venv`) to satisfy ROS 2 Jazzy's package expectations alongside PyTorch.
```bash
# Build the workspace (using symlinks to update python files without rebuilding)
colcon build --symlink-install --packages-select robot_fd

# Source ROS 2 and activate the virtual environment
source install/setup.bash
source .venv/bin/activate
```

### Step 2: Generate Training Data
Launch the pristine simulation to gather baseline residual data. We apply high systemic noise (`0.035`) to ensure the neural net learns to ignore standard physics jitter.
```bash
# Terminal 1 (Run Gazebo)
ros2 launch robot_fd fault_detection.launch.py enable_noise:=true noise_std_dev:=0.035
```
Open a new sourced terminal and run the data recorder. Wait for it to collect 65,000 samples.
```bash
# Terminal 2 (Run Recorder)
source install/setup.bash
source .venv/bin/activate
ros2 run robot_fd data_recorder.py
```

### Step 3: Train the 1D-CNN Autoencoder
With `healthy_residuals.csv` populated, run the offline PyTorch trainer to generate the model weights and threshold.
```bash
# Terminal 2 (Run PyTorch Script)
python3 src/robot_fd/scripts/train_autoencoder.py
```
*(This extracts features over 50-tick sliding windows and outputs weights to the `weights/` directory).*

### Step 4: Run the Full Machine Learning Pipeline
Shut down the previous isolated launch file. Now, start the full orchestration matrix which pipelines corrupted joint states into the live inference engine.
```bash
# Terminal 1 (Run Full Test Architecture)
ros2 launch robot_fd test_fault_detection.launch.py
```
*(The terminal will run silently, processing the healthy sine wave without triggering the anomaly alert).*

### Step 5: Launch the Live Visualizer
Open a new sourced terminal. Start the multi-threaded Matplotlib plotter to track an individual joint. By default it tracks `joint_2`, but you can target any joint (e.g., `joint_4`).
```bash
# Terminal 2 (Run Plotter)
source install/setup.bash
source .venv/bin/activate
ros2 run robot_fd live_plotter.py --ros-args -p target_joint:=joint_4
```

### Step 6: Dynamically Inject Faults!
Open a third sourced terminal. Use standard ROS 2 CLI commands to alter the behavior of `fault_injector.py` in real-time.

**To inject a `0.15` radian bias into `joint_4`:**
```bash
ros2 topic pub -1 /inject_fault sensor_msgs/msg/JointState "{name: ['joint_4'], position: [0.15]}"
```
*   **Observe the Visualizer:** The blue (True) and red (Faulted) lines on the top graph will instantly split. Inside the bottom graph, the orange EKF residual will violently spike.
*   **Observe the Orchestrator:** The `anomaly_detector` node will immediately begin printing `[WARNING]: ANOMALY DETECTED!`asthereconstructionMSEshattersthelearnedthreshold!

**To heal the fault entirely:**
Inject an exact `0.0` offset into the active joint.
```bash
ros2 topic pub -1 /inject_fault sensor_msgs/msg/JointState "{name: ['joint_4'], position: [0.0]}"
```
*(The graphs will converge, and the anomaly warnings will immediately silence).*

You can inject varying magnitudes across multiple joints simultaneously using this dynamic parameter structure!

# Robot Arm Fault Detection (`robot_fd`)

## Overview
The `robot_fd` package is designed for simulating a robotic arm in Gazebo Harmonic using ROS 2 Jazzy, and performing hybrid model-based and data-driven fault detection. The system uses an Extended Kalman Filter (EKF) to generate structured residuals from joint states, which are then analyzed to detect anomalies like joint bias offsets.

Currently, the project focuses on collecting healthy baseline data with simulated joint excitation.

## Nodes Description
* **`sine_wave_commander.py`**: Injects continuous sinusoidal target positions into the `/arm_controller/commands` topic to excite the robot joints. Designed with a phase offset per joint to create a fluid, collision-free workspace motion. Includes optional Gaussian noise injection.
* **`ekf_node.py`**: Subscribes to `/joint_states` and `/arm_controller/commands` and computes state estimates using a simplified 10-dimensional kinematic model. It outputs the difference between predictions and sensor data as residuals to the `/ekf/residuals` topic.
* **`data_recorder.py`**: Subscribes to `/ekf/residuals` and saves the incoming stream to `healthy_residuals.csv`. Automatically stops after collecting the required training samples.
* **`plot_residuals.py`**: A standalone visualization script (using Pandas and Matplotlib) to verify the data distribution of the generated CSV files.
* **`train_autoencoder.py`**: A PyTorch script that loads `healthy_residuals.csv`, trains a 1D-CNN autoencoder to learn the nominal residual patterns, calculates a 99.5% dynamic anomaly threshold, and saves the weights/scaler to `src/robot_fd/weights`.
* **`fault_injector.py`**: Simulates a live sensor/encoder fault by intercepting `/joint_states` and adding a static bias offset to a specific joint (e.g., `joint_2`) after a set delay. Publishes to `/faulty_joint_states`.
* **`anomaly_detector.py`**: The online inference node. Loads the trained PyTorch weights, subscribes to the live `/ekf/residuals` stream, processes them through a sliding window (1D-CNN), and publishes `True` to `/anomaly_detected` if the reconstruction error (MSE) exceeds the learned threshold.

## Launch Files

### `fault_detection.launch.py` (Data Collection)
Launches the Gazebo simulation, the robot state publisher, controllers, the `sine_wave_commander` node, and the `ekf_node`. Use this to collect pristine, uncorrupted data.

**Arguments:**
* `use_sim_time` (default: `true`): Synchronizes the ROS 2 nodes with the Gazebo clock.
* `enable_noise` (default: `false`): Enables Gaussian noise on the sine wave command signal.
* `noise_std_dev` (default: `0.02`): Sets the standard deviation characterizing the intensity of the injected noise.

### `test_fault_detection.launch.py` (Full Pipeline Orchestrator)
Launches the entire fault detection system for live testing. It starts the Gazebo simulation, commander, fault injector, EKF (remapped to read faulty data), and the anomaly detector node all simultaneously.

**Arguments:**
* Includes all arguments from `fault_detection.launch.py` (with defaults set to `enable_noise:=true` and `noise_std_dev:=0.035`).
* `fault_joint_name` (default: `'joint_2'`): The exact joint to inject the bias into.
* `fault_magnitude` (default: `0.1`): The bias offset in radians to simulate a sensor failure.
* `fault_start_time` (default: `10.0`): Delay in simulation seconds before the fault strikes.

## Workflows & Commands

### 1. Build & Setup
*(Ensure you have a Python Virtual Environment initialized with PyTorch and dependencies before sourcing)*
```bash
colcon build --symlink-install --packages-select robot_fd
source install/setup.bash
source .venv/bin/activate
```

### 2. Collect Pristine Training Data
Launch the isolated data collection environment (ensure no fault injector is running). Keep the noise consistent (`0.035`) across both training and testing to prevent false alarms!
```bash
ros2 launch robot_fd fault_detection.launch.py enable_noise:=true noise_std_dev:=0.035
```

Open a new terminal containing `source .venv/bin/activate`, and start recording:
```bash
ros2 run robot_fd data_recorder.py
```
*(Wait until it finishes saving `healthy_residuals.csv`).*

### 3. Train the Autoencoder
Run the standalone PyTorch script to learn the residual patterns. It will evaluate the 99.5% boundary and drop weights into `src/robot_fd/weights`.
```bash
python3 src/robot_fd/scripts/train_autoencoder.py
```

### 4. Run the Live Fault Detection Pipeline
Test everything. Use the integrated orchestrator to launch Gazebo, generate commands (at the exact same noise profile), route signals through the fault injector, process the EKF, and run the online PyTorch detector.
```bash
ros2 launch robot_fd test_fault_detection.launch.py fault_magnitude:=0.1
```
The terminal will remain quiet (except for nominal ROS logs). 
When you are ready to simulate the sudden sensor fault on a specific joint, open a new terminal and run:
```bash
ros2 topic pub -1 /inject_fault sensor_msgs/msg/JointState "{name: ['joint_4'], position: [0.15]}"
```
The node will interpret that as: Add a persistent 0.15 rad offset to joint_4. 
If you want to inject fault on another joint simply run the command again with that joint name. 
If you want to clear the fault simply pass magnitude as zero: 
`ros2 topic pub -1 /inject_fault sensor_msgs/msg/JointState "{name: ['joint_4'], position: [0.0]}"`

Watch the orchestrator terminal. It will immediately print `FAULT INSTRUCTED! Applying <x> override to <y> joint.` closely followed by continuous `ANOMALY DETECTED!` warnings.

## Current Completion Status
* **Phase 1 to Phase 5:** Completed. The 1D-CNN autoencoder successfully classifies healthy vs degraded EKF residual windows dynamically based on a custom `percentile` threshold.

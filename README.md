# Robot Arm Fault Detection (`robot_fd`)

## Overview
The `robot_fd` package is designed for simulating a robotic arm in Gazebo Harmonic using ROS 2 Jazzy, and performing hybrid model-based and data-driven fault detection. The system uses an Extended Kalman Filter (EKF) to generate structured residuals from joint states, which are then analyzed to detect anomalies like joint bias offsets.

Currently, the project focuses on collecting healthy baseline data with simulated joint excitation.

## Nodes Description
* **`sine_wave_commander.py`**: Injects continuous sinusoidal target positions into the `/arm_controller/commands` topic to excite the robot joints. Designed with a phase offset per joint to create a fluid, collision-free workspace motion. Includes optional Gaussian noise injection.
* **`ekf_node.py`**: Subscribes to `/joint_states` and `/arm_controller/commands` and computes state estimates using a simplified 10-dimensional kinematic model. It outputs the difference between predictions and sensor data as residuals to the `/ekf/residuals` topic.
* **`data_recorder.py`**: Subscribes to `/ekf/residuals` and saves the incoming stream to `healthy_residuals.csv`. Automatically stops after collecting the required training samples.
* **`plot_residuals.py`**: A standalone visualization script (using Pandas and Matplotlib) to verify the data distribution of the generated CSV files.

## Launch Files

### `fault_detection.launch.py` (Main Entry Point)
Launches the Gazebo simulation, the robot state publisher, controllers, the `sine_wave_commander` node, and the `ekf_node`.

**Arguments:**
* `use_sim_time` (default: `true`): Synchronizes the ROS 2 nodes with the Gazebo clock.
* `enable_noise` (default: `false`): Enables Gaussian noise on the sine wave command signal.
* `noise_std_dev` (default: `0.02`): Sets the standard deviation characterizing the intensity of the injected noise.

#### Example Usage:
```bash
# Standard simulation (collision-free sine wave, no noise)
ros2 launch robot_fd fault_detection.launch.py

# Simulation with noise enable and custom standard deviation
ros2 launch robot_fd fault_detection.launch.py enable_noise:=true noise_std_dev:=0.05
```

### `gazebo.launch.py` (Base Gazebo Launch)
Spawns the robot in Gazebo Harmonic with `ros2_control` parameters, and initializes the `ros_gz_bridge` for `/clock` and `/joint_states`. Usually called internally by `fault_detection.launch.py`.

## Workflows & Commands

### 1. Build and Source
```bash
colcon build --packages-select robot_fd
source install/setup.bash
```

### 2. Generate and View Data
First, launch the complete environment:
```bash
ros2 launch robot_fd fault_detection.launch.py enable_noise:=true
```

Open a new terminal, and start recording the EKF residuals:
```bash
ros2 run robot_fd data_recorder.py
```
*(Wait until the recorder script automatically terminates after capturing the required samples).*

Plot the results to verify stability, noise profile, and the absence of violent collisions:
```bash
python3 src/robot_fd/scripts/plot_residuals.py
```

## Upcoming Phases
* **Phase 4**: Setup and train the Autoencoder on the generated `healthy_residuals.csv`.
* **Phase 5**: Online fault injection and anomaly detection using the trained autoencoder weights.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_fd = get_package_share_directory('robot_fd')

    # Launch Configurations
    param_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_noise = LaunchConfiguration('enable_noise', default='true')
    noise_std_dev = LaunchConfiguration('noise_std_dev', default='0.035')
    
    # Extracted from fault injector defaults
    fault_joint_name = LaunchConfiguration('fault_joint_name', default='joint_2')
    fault_magnitude = LaunchConfiguration('fault_magnitude', default='0.1')
    fault_start_time = LaunchConfiguration('fault_start_time', default='10.0')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_enable_noise = DeclareLaunchArgument('enable_noise', default_value='true')
    declare_noise_std_dev = DeclareLaunchArgument('noise_std_dev', default_value='0.035')
    
    declare_fault_joint_name = DeclareLaunchArgument('fault_joint_name', default_value='joint_2')
    declare_fault_magnitude = DeclareLaunchArgument('fault_magnitude', default_value='0.1')
    declare_fault_start_time = DeclareLaunchArgument('fault_start_time', default_value='10.0')

    # Include Gazebo Simulation Launch File
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_fd, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': param_sim_time}.items()
    )

    # Sine Wave Commander Node
    sine_wave_commander_node = Node(
        package='robot_fd',
        executable='sine_wave_commander.py',
        name='sine_wave_commander',
        output='screen',
        parameters=[{
            'use_sim_time': param_sim_time,
            'enable_noise': enable_noise,
            'noise_std_dev': noise_std_dev
        }]
    )

    # Fault Injector Node (Listens to /joint_states from Gazebo, outputs to /faulty_joint_states)
    fault_injector_node = Node(
        package='robot_fd',
        executable='fault_injector.py',
        name='fault_injector',
        output='screen',
        parameters=[{
            'use_sim_time': param_sim_time,
            'fault_joint_name': fault_joint_name,
            'fault_magnitude': fault_magnitude,
            'fault_start_time': fault_start_time
        }]
    )

    # EKF Node
    # Remap /joint_states to /faulty_joint_states so EKF processes the corrupted data stream
    ekf_node = Node(
        package='robot_fd',
        executable='ekf_node.py',
        name='ekf_node',
        output='screen',
        remappings=[
            ('/joint_states', '/faulty_joint_states')
        ],
        parameters=[{
            'use_sim_time': param_sim_time
        }]
    )

    # Anomaly Detector Node (Live PyTorch Inference)
    anomaly_detector_node = Node(
        package='robot_fd',
        executable='anomaly_detector.py',
        name='anomaly_detector',
        output='screen',
        parameters=[{
            'use_sim_time': param_sim_time,
            'model_dir': 'src/robot_fd/weights'  # Explicit path for development
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_noise,
        declare_noise_std_dev,
        declare_fault_joint_name,
        declare_fault_magnitude,
        declare_fault_start_time,
        
        gazebo_launch,
        sine_wave_commander_node,
        fault_injector_node,
        ekf_node,
        anomaly_detector_node,
    ])
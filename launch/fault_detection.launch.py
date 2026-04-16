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
    enable_noise = LaunchConfiguration('enable_noise', default='false')
    noise_std_dev = LaunchConfiguration('noise_std_dev', default='0.02')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_enable_noise = DeclareLaunchArgument(
        'enable_noise',
        default_value='false',
        description='Enable Gaussian noise on sine wave commands'
    )
    
    declare_noise_std_dev = DeclareLaunchArgument(
        'noise_std_dev',
        default_value='0.02',
        description='Standard deviation of the Gaussian noise'
    )

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

    # EKF Node
    ekf_node = Node(
        package='robot_fd',
        executable='ekf_node.py',
        name='ekf_node',
        output='screen',
        parameters=[{
            'use_sim_time': param_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_noise,
        declare_noise_std_dev,
        gazebo_launch,
        sine_wave_commander_node,
        ekf_node,
    ])

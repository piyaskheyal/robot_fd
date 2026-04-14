import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    pkg_name = 'robot_fd'
    pkg_share = get_package_share_directory(pkg_name)

    # Process URDF file
    urdf_file = os.path.join(pkg_share, 'model', 'robot_arm_urdf.xacro')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}
    
    param_sim_time = {"use_sim_time": True}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, param_sim_time]
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot_arm',
                   '-z', '0.0']
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Arm Controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # Gripper Controller
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # ROS-GZ Bridge for /clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
    ])

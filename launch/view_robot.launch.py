import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotXacroName='robot_arm_urdf'
    namePackage='robot_fd'
    modelFileRelativePath='model/robot_arm_urdf.xacro'
    pathModelFile=os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    param_sim_time = {"use_sim_time": False}
    
    robotDescription=xacro.process_file(pathModelFile).toxml()
    
    nodeRobotStatePublisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robotDescription},
            param_sim_time,
        ],
    )

    nodeRobotJointStatePublisher=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rvizConfigFile=os.path.join(get_package_share_directory(namePackage),'config/rviz_robot_arm.rviz')
    nodeRviz2=Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rvizConfigFile],
    )


    launchDescriptionObject=LaunchDescription()
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(nodeRobotJointStatePublisher)
    launchDescriptionObject.add_action(nodeRviz2)

    return launchDescriptionObject
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'arm_description'
    
    pkg_share_path = os.path.join(get_package_share_directory(pkg_name))

    xacro_file = os.path.join(pkg_share_path, 'urdf', 'meu_braco.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    params = {'robot_description': robot_description}

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_config_file = os.path.join(pkg_share_path, 'rviz', 'view_robot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz_node,
    ])
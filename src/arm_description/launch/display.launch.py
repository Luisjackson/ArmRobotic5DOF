# arm_description/launch/display.launch.py

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_path = launch_ros.substitutions.FindPackageShare(package='arm_description').find('arm_description')
    
    urdf_model_path = os.path.join(pkg_path, 'urdf', 'meu_braco.urdf.xacro')

    with open(urdf_model_path, 'r') as infp:
        robot_desc_raw = infp.read()
    
    robot_description_params = {
        'robot_description': Command(['xacro ', urdf_model_path])
    }

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'display.rviz')] 
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='true',
                                             description='Flag to enable joint state publisher GUI'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_model_path,
                                             description='Path to the URDF model'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
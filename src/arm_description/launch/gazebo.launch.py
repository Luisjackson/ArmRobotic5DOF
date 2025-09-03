import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_dir = get_package_share_directory('arm_description')
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pkg_dir, 'urdf', 'meu_braco.urdf.xacro'),
        description='Caminho absoluto para o arquivo URDF do robô'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True} 
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'meu_braco'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['braco_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garra_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])
# braco_moveit_config/launch/bringup_robot.launch.py (Corrigido)

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():

    robot_control_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "real_robot.ros2_control.xacro"]
    )

    # Processa o XACRO, injetando o caminho para o arquivo de controle
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_description"), "urdf", "meu_braco.urdf.xacro"]
            ),
            " ",
            "ros2_control_xacro_path:=", robot_control_file,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Carrega o arquivo de configuração dos controladores
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("braco_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Nó do ros2_control (controller_manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers], 
        output="screen",
    )

    # Nó do Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': False}], 
        
    )

    # Spawners dos controladores
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
        )
        for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=spawner_nodes,
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        delayed_spawners,
    ])

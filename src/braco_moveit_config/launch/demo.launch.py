# braco_moveit_config/launch/demo.launch.py (RÉPLICA DO ANTIGO)

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Carrega a configuração do MoveIt, incluindo o URDF e os controladores
    moveit_config = (
        MoveItConfigsBuilder("turtlebot_arm", package_name="braco_moveit_config")
        .robot_description(
            file_path="config/turtlebot_arm.urdf.xacro",
            # Adicionado para passar os argumentos que faltavam
            mappings={
                "is_ignition": "false",
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Nó do ros2_control (que vai carregar o FakeSystem/MockSystem da simulação)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                FindPackageShare("braco_moveit_config").find("braco_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="screen",
    )

    # Nó do Robot State Publisher (explícito como no projeto antigo)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # Nó do RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Nó do MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # Spawners que serão iniciados DEPOIS do ros2_control_node
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
            output="screen",
        ) for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    # Event handler para garantir a ordem de inicialização
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawner_nodes,
        )
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher_node,
            move_group_node,
            ros2_control_node,
            delayed_spawners,
        ]
    )
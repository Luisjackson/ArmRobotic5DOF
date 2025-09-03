# braco_moveit_config/launch/real_robot.launch.py (VERSÃO FINAL)

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Construir o caminho absoluto para o arquivo URDF no pacote 'arm_description'
    urdf_path = os.path.join(
        get_package_share_directory("arm_description"),
        "urdf",
        "meu_braco.urdf.xacro",
    )

    # Usar o MoveItConfigsBuilder, passando o caminho absoluto para o URDF
    moveit_config = (
        MoveItConfigsBuilder("meu_braco", package_name="braco_moveit_config")
        .robot_description(file_path=urdf_path, mappings={"use_real_hardware": "true"})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # Nó do MoveGroup (para planejamento)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz (para visualização e comando)
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
        ],
    )

    # Nó do ros2_control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )

    # Spawners para carregar os controladores
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
        )
        for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    # Garante que os spawners iniciem após o ros2_control_node
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawner_nodes,
        )
    )

    return LaunchDescription(
        [
            rviz_node,
            move_group_node,
            ros2_control_node,
            delayed_spawners,
        ]
    )
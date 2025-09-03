# braco_moveit_config/launch/moveit_rviz.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Pega o caminho absoluto para o arquivo URDF no pacote 'arm_description'
    urdf_path = os.path.join(
        get_package_share_directory("arm_description"),
        "urdf",
        "meu_braco.urdf.xacro",
    )

    # Usa o MoveItConfigsBuilder para carregar a configuração
    moveit_config = (
        MoveItConfigsBuilder("meu_braco", package_name="braco_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "use_real_hardware": "true",
                "is_ignition": "false",
            },
        )   
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Prepara os parâmetros para o move_group, adicionando a configuração de tempo
    move_group_params = moveit_config.to_dict()
    move_group_params['use_sim_time'] = False 

    # Nó do MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params], 
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
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
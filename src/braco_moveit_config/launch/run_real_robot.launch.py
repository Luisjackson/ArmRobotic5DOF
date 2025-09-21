from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Construir o caminho absoluto para o arquivo URDF
    urdf_path = os.path.join(
        get_package_share_directory("arm_description"),
        "urdf",
        "meu_braco.urdf.xacro",
    )

    # Usar o MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("meu_braco", package_name="braco_moveit_config")
        .robot_description(file_path=urdf_path, mappings={"use_real_hardware": "true"})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # Parâmetro explícito para usar o relógio do sistema (wall time)
    use_sim_time_param = {"use_sim_time": False}

    # Nó do MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # Adiciona o parâmetro de tempo aos outros parâmetros do MoveIt
        parameters=[moveit_config.to_dict(), use_sim_time_param],
    )

    # RViz
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
            use_sim_time_param, 
        ],
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time_param], # Adiciona o parâmetro de tempo
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

    # Spawners
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
            target_action=ros2_control_node,
            on_start=spawner_nodes,
        )
    )

    # static_tf_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_base_to_camera',
    #     arguments=[
    #         # x, y, z, yaw, pitch, roll
    #         '0.2', '0.0', '0.5', '0', '0', '0', # Exemplo: 20cm para a frente, 50cm para cima
    #         'base_link', # Frame pai (a base do robô)
    #         'camera_link'  # Frame filho (a sua câmara)
    #     ]
    # )

    return LaunchDescription(
        [
            rviz_node,
            move_group_node,
            robot_state_publisher_node,
            ros2_control_node,
            delayed_spawners,
            # static_tf_publisher 
        ]
    )


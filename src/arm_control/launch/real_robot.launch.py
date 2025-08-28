# arm_control/launch/real_robot.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Caminho para o URDF do robô
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "meu_braco.urdf.xacro"]),
            " ",
            "use_real_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Caminho para o arquivo de controladores
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("arm_control"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Nó do Controller Manager (ros2_control)
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
        parameters=[robot_description],
    )

    # Spawners dos controladores
    # Garante que eles só iniciem DEPOIS que o control_node estiver ativo
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Inicia os spawners em sequência após o control_node
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner, arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        delayed_spawners,
    ])
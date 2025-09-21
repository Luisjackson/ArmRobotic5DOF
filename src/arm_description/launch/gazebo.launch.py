import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_dir = get_package_share_directory('arm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Caminho para o seu ficheiro URDF principal que contém a câmara
    xacro_file = os.path.join(pkg_dir, 'urdf', 'meu_braco.urdf.xacro')

    # Processa o ficheiro XACRO para gerar a descrição do robô
    # Passamos 'use_real_hardware:=false' para garantir que os plugins de simulação sejam carregados
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_real_hardware:=false', # Argumento que você já tinha
        ' is_ignition:=true',
        ' use_gazebo_uri:=true'    
    ])

    # Nó do Robot State Publisher (publica as transformações do robô)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            {'use_sim_time': True} 
        ]
    )

    # Inicia o mundo do Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Pode mudar 'empty.sdf' para outro mundo se tiver um
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Insere o robô no Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'meu_braco'],
        output='screen'
    )

    # Inicia os controladores do braço no Gazebo para que você possa movê-lo
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])
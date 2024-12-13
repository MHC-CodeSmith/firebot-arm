from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
    # Define o diretório do pacote
    share_dir = FindPackageShare('firebot_arm_pkg').find('firebot_arm_pkg')

    # Processa o arquivo Xacro para gerar o modelo URDF do robô
    xacro_file = os.path.join(share_dir, 'urdf', 'montagem.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()  # Converte o conteúdo Xacro em XML

    # Publica o robot_description como um tópico
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],  # Publica o URDF
        output='screen'
    )

    # Publica os estados das juntas
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Inicia o servidor Gazebo
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'pause': 'true'}.items()
    )

    # Inicia o cliente Gazebo
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Configuração para spawnar o robô no ambiente Gazebo
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'montagem', '-topic', 'robot_description'],
        output='screen'
    )

    # Retorna a descrição do lançamento contendo todos os nós e ações definidos
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define o diretório do pacote
    share_dir = FindPackageShare('firebot_arm_pkg').find('firebot_arm_pkg')

    # Processa o arquivo Xacro para gerar o modelo URDF do robô
    xacro_file = os.path.join(share_dir, 'urdf', 'montagem.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()  # Converte o conteúdo Xacro em XML

    # Configuração do nó robot_state_publisher
    # Publica os estados do robô (como links e juntas) para o ROS 2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}  # Define o modelo URDF a ser usado
        ],
        output='screen'
    )

    # Configuração do nó joint_state_publisher
    # Publica os estados das juntas, útil para simulação estática
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Inicia o servidor Gazebo (simulação física)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'  # Arquivo de inicialização do servidor Gazebo
            ])
        ]),
        launch_arguments={'pause': 'true'}.items()  # Inicia a simulação pausada
    )

    # Inicia o cliente Gazebo (interface gráfica para a simulação)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'  # Arquivo de inicialização do cliente Gazebo
            ])
        ])
    )

    # Configuração para spawnar o robô no ambiente Gazebo
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'montagem',  # Nome da entidade (robô)
            '-topic', 'robot_description'  # Fonte do modelo URDF
        ],
        output='screen'
    )

    # Configuração do nó para gerenciar controladores usando ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_urdf},  # Passa o modelo URDF para o controlador
            os.path.join(share_dir, 'config', 'controllers.yaml')  # Configuração dos controladores
        ],
        output='screen'
    )

    # Spawner para o controlador de estado das juntas
    spawner_joint_state_controller = TimerAction(
        period=2.0,  # Aguarda 2 segundos antes de iniciar
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ],
    )

    # Spawner para o controlador de trajetória das juntas
    spawner_joint_trajectory_controller = TimerAction(
        period=4.0,  # Aguarda 4 segundos antes de iniciar
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ],
    )

    # Retorna a descrição do lançamento contendo todos os nós e ações definidos
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        controller_manager,
        spawner_joint_state_controller,
        spawner_joint_trajectory_controller,
    ])

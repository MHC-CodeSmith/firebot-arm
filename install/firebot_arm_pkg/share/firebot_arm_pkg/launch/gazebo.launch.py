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
    # Diretório do pacote
    share_dir = FindPackageShare('firebot_arm_pkg').find('firebot_arm_pkg')

    # Processar o arquivo Xacro e gerar o URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'montagem.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Nó do robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ],
        output='screen'
    )

    # Nó do joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Iniciar o Gazebo Server
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

    # Iniciar o Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawnar o robô no Gazebo
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'montagem',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Nó para gerenciar controladores com ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_urdf},
            os.path.join(share_dir, 'config', 'controllers.yaml')  # Use o caminho correto para controllers.yaml
        ],
        output='screen'
    )


    spawner_joint_state_controller = TimerAction(
        period=2.0,  # 2 segundos de espera
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ],
    )

    spawner_joint_trajectory_controller = TimerAction(
        period=4.0,  # 4 segundos de espera
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ],
    )

    # Retornar a descrição do lançamento
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

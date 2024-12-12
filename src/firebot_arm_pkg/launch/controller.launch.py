from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Inicializa o ros2_control_node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            '/home/developer/ARM_ws/src/firebot_arm_pkg/config/controllers.yaml'
        ],
        output='screen'
    )

    # Spawner para o controlador de estado das juntas
    spawner_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner para o controlador de trajetória das juntas
    spawner_trajectory_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['trajectory_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        controller_manager,
        spawner_joint_state_broadcaster,
        spawner_trajectory_position_controller,
    ])

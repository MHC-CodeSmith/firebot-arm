from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtém o diretório do pacote 'firebot_arm_pkg'
    share_dir = get_package_share_directory('firebot_arm_pkg')

    # Caminho do arquivo Xacro que define o modelo do robô
    xacro_file = os.path.join(share_dir, 'urdf', 'montagem.xacro')
    # Processa o arquivo Xacro para gerar a descrição do robô em formato URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()  # Converte a configuração para XML

    # Caminho do arquivo de configuração do RViz para visualização
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Declaração de argumento para ativar ou desativar a interface gráfica (GUI)
    gui_arg = DeclareLaunchArgument(
        name='gui',  # Nome do argumento
        default_value='True'  # Valor padrão: ativar GUI
    )

    # Configuração que determina se a GUI será exibida
    show_gui = LaunchConfiguration('gui')

    # Nó responsável por publicar o estado do robô baseado na descrição URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',  # Pacote ROS responsável por publicar estados
        executable='robot_state_publisher',  # Executável do nó
        name='robot_state_publisher',  # Nome do nó
        parameters=[
            {'robot_description': robot_urdf}  # Adiciona a descrição do robô
        ]
    )

    # Nó para publicar estados das juntas sem interface gráfica (usado quando GUI está desativada)
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),  # Executa somente se GUI estiver desativada
        package='joint_state_publisher',  # Pacote ROS para publicar estados das juntas
        executable='joint_state_publisher',  # Executável do nó
        name='joint_state_publisher'  # Nome do nó
    )

    # Nó para publicar estados das juntas com interface gráfica (usado quando GUI está ativada)
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),  # Executa somente se GUI estiver ativada
        package='joint_state_publisher_gui',  # Pacote ROS para GUI de estados das juntas
        executable='joint_state_publisher_gui',  # Executável do nó
        name='joint_state_publisher_gui'  # Nome do nó
    )

    # Nó para iniciar o RViz com a configuração especificada
    rviz_node = Node(
        package='rviz2',  # Pacote ROS para visualização em RViz
        executable='rviz2',  # Executável do RViz
        name='rviz2',  # Nome do nó
        arguments=['-d', rviz_config_file],  # Define o arquivo de configuração do RViz
        output='screen'  # Exibe a saída no terminal
    )

    # Retorna a descrição completa do lançamento, incluindo todos os nós e argumentos
    return LaunchDescription([
        gui_arg,  # Argumento para GUI
        robot_state_publisher_node,  # Nó para publicar o estado do robô
        joint_state_publisher_node,  # Nó para estados das juntas sem GUI
        joint_state_publisher_gui_node,  # Nó para estados das juntas com GUI
        rviz_node  # Nó para visualização no RViz
    ])

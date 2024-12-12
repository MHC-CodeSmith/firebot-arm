from setuptools import setup

# Nome do pacote ROS 2
package_name = 'firebot_arm_pkg'

# Configuração do pacote usando setuptools
setup(
    name=package_name,  # Nome do pacote
    version='0.0.1',  # Versão inicial do pacote
    packages=[],  # Não há pacotes Python, apenas scripts
    data_files=[  # Arquivos adicionais que serão instalados junto com o pacote
        # Registra o pacote no índice de recursos do ROS 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Adiciona o arquivo package.xml que descreve o pacote
        ('share/' + package_name, ['package.xml']),
        # Inclui os arquivos de lançamento (launch files) para ROS 2
        ('share/' + package_name + '/launch', ['launch/display.launch.py', 'launch/gazebo.launch.py']),
        # Inclui arquivos de configuração, como RViz e controladores
        ('share/' + package_name + '/config', ['config/display.rviz', 'config/controllers.yaml']),
        # Inclui os modelos URDF/Xacro necessários para a simulação do robô
        ('share/' + package_name + '/urdf', [
            'urdf/montagem.xacro',  # Arquivo principal do robô em formato Xacro
            'urdf/montagem.trans',  # Configuração de transmissões para o robô
            'urdf/montagem.gazebo',  # Configuração do modelo para simulação no Gazebo
            'urdf/materials.xacro',  # Definições de materiais usados no modelo
            'urdf/montagem.control.xacro',  # Configuração dos controladores do robô
        ]),
        # Adiciona os arquivos de malha (STL) para visualização do robô
        ('share/' + package_name + '/meshes', [
            'meshes/base_link.stl',  # Malha da base do robô
            'meshes/linkvertical1_1.stl',  # Malha do primeiro link vertical
            'meshes/linkhorizontal1_1.stl',  # Malha do link horizontal
            'meshes/linkvertical2_1.stl',  # Malha do segundo link vertical
        ]),
    ],
    scripts=[
        'scripts/direct_kinematics.py',  # Referência ao script direto
        'scripts/inverse_kinematics.py',  # Referência ao script inverso
    ],
    install_requires=['setuptools'],  # Dependência necessária para instalação
    zip_safe=True,  # Indica que o pacote pode ser distribuído como um arquivo zip
    maintainer='MHC',  # Nome do responsável pela manutenção do pacote
    maintainer_email='carvalho.matheush@gmail.com',  # Email de contato do mantenedor
    description='Pacote de simulação do manipulador Firebot Arm',  # Descrição do pacote
    license='MIT',  # Licença do pacote
    tests_require=['pytest'],  # Dependências para rodar testes

    entry_points={
        'console_scripts': [
            'direct_kinematics = direct_kinematics:main',  # Referência correta ao script direto
            'inverse_kinematics = inverse_kinematics:main',  # Referência correta ao script inverso
        ],
    },
)

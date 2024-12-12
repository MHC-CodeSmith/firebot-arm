from setuptools import setup

package_name = 'firebot_arm_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py', 'launch/gazebo.launch.py']),
        ('share/' + package_name + '/config', ['config/display.rviz', 'config/controllers.yaml']),
        ('share/' + package_name + '/urdf', [
            'urdf/montagem.xacro',
            'urdf/montagem.trans',
            'urdf/montagem.gazebo',
            'urdf/materials.xacro',
            'urdf/montagem.control.xacro',
        ]),
        ('share/' + package_name + '/meshes', [
            'meshes/base_link.stl',
            'meshes/linkvertical1_1.stl',
            'meshes/linkhorizontal1_1.stl',
            'meshes/linkvertical2_1.stl',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='Pacote de simulação do manipulador Firebot Arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

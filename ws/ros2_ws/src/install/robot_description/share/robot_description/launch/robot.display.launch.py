import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Diretórios do pacote
    package_share = get_package_share_directory('robot_description')

    # Configurações do Launch
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')
    calibration_file = LaunchConfiguration('calibration_file')

    # Declaração dos argumentos
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_share, 'rviz', 'robot.rviz'),
        description='Caminho completo para o arquivo de configuração do RViz')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Iniciar o Robot State Publisher')

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Iniciar o Joint State Publisher com GUI')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Iniciar o RViz')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(package_share, 'xacro', 'robot.urdf.xacro'),
        description='Caminho completo para o arquivo URDF/XACRO do robô')

    declare_calibration_file_cmd = DeclareLaunchArgument(
        'calibration_file',
        default_value=os.path.join(package_share, 'config', 'calibration.yaml'),
        description='Caminho completo para o arquivo de calibração YAML')

    # Nodes condicionais
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', urdf_file,
                ' calibration_file:=', calibration_file
            ])
        }]
    )

    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Criação do LaunchDescription
    ld = LaunchDescription()

    # Adiciona os argumentos
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_calibration_file_cmd)

    # Adiciona os Nodes
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld

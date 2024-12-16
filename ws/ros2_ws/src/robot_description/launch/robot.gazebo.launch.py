import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Diretório do pacote
    package_share = get_package_share_directory('robot_description')

    # Configurações do Launch
    model_file = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declaração dos argumentos
    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(package_share, 'xacro', 'robot.urdf.xacro'),
        description='Caminho completo para o arquivo XACRO do robô'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Usar tempo simulado (sim_time)'
    )

    # Configurando o ambiente do Gazebo para encontrar as malhas
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(package_share, 'meshes')
    )

    # Descrição do robô gerada pelo XACRO
    robot_description = ParameterValue(
        Command(['xacro ', model_file]),
        value_type=str
    )

    # Publica o estado do robô no ROS 2
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}]
    )

    # Inclui o lançamento do Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    # Spawna o robô no Gazebo usando o ros_gz_sim
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "robot"]
    )

    # Ponte ROS-Gazebo para tempo simulado
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msg.Clock]"],
        output="screen"
    )

    # Criação do LaunchDescription
    ld = LaunchDescription()

    # Adiciona as variáveis de ambiente
    ld.add_action(set_gazebo_model_path)

    # Adiciona os argumentos
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Adiciona os nós e descrições
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_launch)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    return ld

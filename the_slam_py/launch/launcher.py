from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Argumentos de lançamento para o SLAM e o Fixed Frame
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar o tempo simulado para SLAM (simulação ou não)'
        ),
        DeclareLaunchArgument(
            'use_map',
            default_value='false',
            description='Define se o robô deve carregar um mapa existente'
        ),

        # Inicia o Cartographer para SLAM
        Node(
            package='turtlebot3_cartographer',
            executable='cartographer',
            name='cartographer',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/scan', '/turtlebot3/scan'),  # Certifique-se de que o tópico do laser scanner está correto
                ('/odom', '/odom')
            ]
        ),

        # Nó do mapa ocupacional para Navegação
        Node(
            package='nav2_map_server',
            executable='map_server_cli',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=LaunchConfiguration('use_map')  # Condicional baseado no uso de um mapa existente
        ),

        # Nó personalizado para navegação programática
        Node(
            package='the_slam_py',
            executable='shazam',
            name='shazam',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

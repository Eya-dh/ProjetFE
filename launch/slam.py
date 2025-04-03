from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Chemin vers le fichier URDF du robot
    urdf_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'urdf',
        'robot.urdf'
    )
    
    # Vérifier si le fichier URDF existe
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"Le fichier URDF est introuvable : {urdf_path}")
    
    # Lire le contenu de l'URDF
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    
    # Chemin vers le fichier de configuration des contrôleurs
    controllers_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'config',
        'summit_controllers.yaml'
    )
    # Modifier la configuration SLAM Toolbox
    slam_toolbox_config_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'config',
        'slam_toolbox_config.yaml'
    )


    # Chemin vers le fichier de configuration RViz
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'rviz_config',
        'slam_config.rviz'
    )
    
    # Chemin vers l'environnement hospital_.world
    world_file_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'worlds',
        'newhospital.world'
    )
    
    # Vérifier si le fichier de monde existe
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"Le fichier world est introuvable : {world_file_path}")

    # Déclarer l'argument pour utiliser le temps de simulation
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation (true/false)'
    )
    
    # Lancer Gazebo avec l'environnement hospital_.world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # Créer l'action pour spawner l'entité
    gazebo_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
                    '-entity', 'robot_hospital',
                    '-topic', 'robot_description',
                    '-x', '1.0',
                    '-y', '1.0',
                    '-z', '0.0',
                    '-Y', '0.0'
                ],
        output='screen'
    )
    
    # Ajouter un délai de 5 secondes avant le spawn
    delayed_spawn = TimerAction(
        period=5.0,  # Délai en secondes
        actions=[gazebo_spawn]
    )
    
    # Publier l'URDF dans `/robot_description`
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Publier les états des joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Nœud SLAM Toolbox pour la cartographie
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            slam_toolbox_config_path
            
        ]
    )
    
    # Lancer RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        robot_state_publisher,  # Lancer le robot_state_publisher avant le spawn
        joint_state_publisher,
        delayed_spawn,  # Utiliser la version avec délai
        slam_toolbox_node,
        rviz_node
    ])
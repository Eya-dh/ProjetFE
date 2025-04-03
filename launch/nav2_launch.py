from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    # Chemin vers le fichier de configuration RViz
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_hospital_logistics'),
        'rviz_config',
        'summit_config.rviz'
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

    # Charger le robot dans Gazebo
    gazebo_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_hospital', '-file', urdf_path],        
        output='screen'
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
    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[controllers_path],
    output='screen'
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

    # Lancer Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': '/home/eya/ros2_ws/src/robot_hospital_logistics/hospital_map/maps.yaml',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{'yaml_filename': '/home/eya/ros2_ws/src/robot_hospital_logistics/hospital_map/maps.yaml'}]
    )

    map_to_odom = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    output='screen'
)


    return LaunchDescription([
        declare_use_sim_time,  # Ajouter l'argument pour use_sim_time
        gazebo_launch,
        gazebo_spawn,
        robot_state_publisher,
        joint_state_publisher,
        nav2_bringup,
        map_server_node,
        controller_manager,map_to_odom
    ])
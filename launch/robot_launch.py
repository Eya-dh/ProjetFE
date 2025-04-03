from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Lancer Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
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
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Publier les états des joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )


    controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', 'robot_controller'],
    output='screen'
)


    # Lancer RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        gazebo_spawn,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        controller_spawner
    ])

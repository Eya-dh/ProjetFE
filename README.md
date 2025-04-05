# Se placer dans le dossier source du workspace ROS2
cd ~/ros2_ws/src

# Cloner le dépôt Git contenant le projet Hospitalbot Path Planning
git clone https://github.com/TommasoVandermeer/Hospitalbot-Path-Planning.git

# Se rendre dans le dossier du package contenant les modèles et photos pour Gazebo
cd ~/ros2_ws/src/Hospitalbot-Path-Planning/hospital_robot_spawner

# Copier tous les modèles Gazebo dans le dossier par défaut de Gazebo (~/.gazebo/models)
cp -r models/. ~/.gazebo/models

# Copier les photos utilisées dans l’environnement Gazebo dans le bon dossier (~/.gazebo/photos)
cp -r photos/. ~/.gazebo/photos

# Lancer le nœud de caméra (lecture des QR codes ou visualisation via RealSense)
ros2 launch robot_hospital_logistics hospital_cam.py

# Lancer la navigation avec Nav2 en chargeant la carte (maps.yaml)
ros2 launch nav2_bringup bringup_launch.py map:=maps.yaml

# Ouvrir RViz2 avec la configuration par défaut de Nav2 pour visualiser la navigation
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

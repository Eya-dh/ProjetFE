Ce code installe les modèles et photos nécessaires à la simulation de l’environnement hospitalier dans Gazebo.
cd ~/ros2_ws/src

git clone https://github.com/TommasoVandermeer/Hospitalbot-Path-Planning.git

cd ~/ros2_ws/src/Hospitalbot-Path-Planning/hospital_robot_spawner

cp -r models/. ~/.gazebo/models

cp -r photos/. ~/.gazebo/photos

ros2 launch robot_hospital_logistics hospital_cam.py

ros2 launch nav2_bringup bringup_launch.py map:=maps.yaml

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

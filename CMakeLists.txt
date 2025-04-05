cmake_minimum_required(VERSION 3.8)
project(robot_hospital_logistics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install worlds, launch files et urdf
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})# Installer les fichiers  RViz, et Config
install(DIRECTORY rviz_config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# Install the URDF directory
install(DIRECTORY urdf config rviz_config
  DESTINATION share/${PROJECT_NAME})

# Install directories
install(DIRECTORY 
  launch
  urdf
  rviz_config
  hospital_map

  DESTINATION share/${PROJECT_NAME}/
)





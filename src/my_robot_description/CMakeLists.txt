cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Robot data is shared by ROS tools and the independent MuJoCo preview.
install(DIRECTORY urdf mjcf meshes rviz DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})

ament_package()

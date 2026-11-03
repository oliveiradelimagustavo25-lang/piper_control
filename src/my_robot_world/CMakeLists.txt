cmake_minimum_required(VERSION 3.8)
project(my_robot_world)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR .)
install(DIRECTORY objects basic assets config DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" EXCLUDE PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()

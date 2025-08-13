cmake_minimum_required(VERSION 3.5)
project(ouster_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY meshes urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

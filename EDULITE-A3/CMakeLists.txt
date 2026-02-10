cmake_minimum_required(VERSION 3.8)
project(edulite_a3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES EDULITE-A3.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

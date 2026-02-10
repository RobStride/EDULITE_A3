cmake_minimum_required(VERSION 3.8)
project(el_a3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES EL-A3.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(el_a3_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


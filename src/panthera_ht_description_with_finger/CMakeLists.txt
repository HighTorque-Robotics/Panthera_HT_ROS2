cmake_minimum_required(VERSION 3.8)
project(panthera_ht_description_with_finger)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  textures
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

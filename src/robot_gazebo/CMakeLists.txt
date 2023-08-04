cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Set project name
project(robot_gazebo)

# Setup
find_package(ignition-cmake2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ignition-sensors6 REQUIRED)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

ign_find_package(ignition-sensors6 REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})

# Add src
add_subdirectory(src)

# Export all worlds
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Finish
ament_package()
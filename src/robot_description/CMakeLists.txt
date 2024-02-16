cmake_minimum_required(VERSION 3.5)

project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Download all models
install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Download ROV model
# install(
#   DIRECTORY
#    model/bluerov2_heavy/model.sdf
#    DESTINATION share/${PROJECT_NAME}/model/bluerov2_heavy.xacro
# )

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
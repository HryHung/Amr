cmake_minimum_required(VERSION 3.8)
project(amr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY urdf launch meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wheel_rpm_odometry.py
  scripts/bno055_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.5)
project(mjrbot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# MODIFIED 1/26/2025
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
# MODIFIED 1/26/2025

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds nodes
  DESTINATION share/${PROJECT_NAME}
)

# MODIFIED 1/26/2025

# Add Python scripts installation
install(PROGRAMS
  nodes/arduino_serial_node.py
  nodes/sensor_navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes with execution permission
install(
  PROGRAMS
    nodes/pose_capture.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_capture
)

# Install Python nodes with execution permission
install(
  PROGRAMS
    nodes/light_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME light_navigation
)

# Install Python nodes with execution permission
install(
  PROGRAMS
    nodes/rain_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rain_navigation
)


# MODIFIED 1/26/2025

ament_package()

cmake_minimum_required(VERSION 3.8)
project(cansat_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Find Gazebo libraries
find_package(gazebo REQUIRED)
find_package(sensor_msgs REQUIRED)
# Build the plugin as a shared library
# Build the plugin as a shared library
add_library(aerodynamics_plugin SHARED src/aerodynamics_plugin.cpp)
target_include_directories(aerodynamics_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(aerodynamics_plugin ${GAZEBO_LIBRARIES})

# Build the controller node executable
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp sensor_msgs)

# Install the executable
install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin library to the correct location
install(TARGETS aerodynamics_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Build the reaction wheel plugin
add_library(reaction_wheel_plugin SHARED src/reaction_wheel_plugin.cpp)
ament_target_dependencies(reaction_wheel_plugin rclcpp std_msgs gazebo)
target_include_directories(reaction_wheel_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(reaction_wheel_plugin ${GAZEBO_LIBRARIES})

# Install the plugin library
install(TARGETS reaction_wheel_plugin
  DESTINATION lib/${PROJECT_NAME}
)
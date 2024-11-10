cmake_minimum_required(VERSION 3.8)
project(joint_controller_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp 
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  parameter_traits
  realtime_tools
  joint_controller_msgs
  sensor_msgs

)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_controller_parameters
  src/joint_controller_parameters.yaml
)

# Joint Controller Core library //TODO: CHANGE THIS TO SEPERATE PACKAGE!

add_library(joint_controller SHARED
  src/joint_controller_core.cpp
  src/pid_controller/pid_controller.cpp
)
target_include_directories(joint_controller
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")


# Joint Controller Interface library  
add_library(
  joint_controller_interface
  SHARED
  src/joint_controller_interface.cpp
)
target_include_directories(joint_controller_interface
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_link_libraries(
  joint_controller_interface
  PUBLIC joint_controller
  PUBLIC joint_controller_parameters
)

ament_target_dependencies(
  joint_controller_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY include/
  DESTINATION include/joint_controller_ros2_control
)
install(TARGETS
    joint_controller_interface
    joint_controller_parameters
  EXPORT joint_controller_ros2_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()

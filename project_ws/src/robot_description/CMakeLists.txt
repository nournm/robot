cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(fake_lidar_pub src/fake_lidar_pub.cpp)
ament_target_dependencies(fake_lidar_pub rclcpp sensor_msgs)
install(TARGETS fake_lidar_pub DESTINATION lib/${PROJECT_NAME})

add_executable(fake_lidar_sub src/fake_lidar_sub.cpp)
ament_target_dependencies(fake_lidar_sub rclcpp sensor_msgs)
install(TARGETS fake_lidar_sub DESTINATION lib/${PROJECT_NAME})

# Install launch directory
install(
  DIRECTORY launch meshes urdf config worlds rviz maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

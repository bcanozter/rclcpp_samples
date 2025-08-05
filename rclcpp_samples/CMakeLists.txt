cmake_minimum_required(VERSION 3.8)
project(rclcpp_samples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(custom_qos_publisher src/custom_qos_publisher.cpp)
ament_target_dependencies(custom_qos_publisher rclcpp std_msgs)

add_executable(custom_qos_subscriber src/custom_qos_subscriber.cpp)
ament_target_dependencies(custom_qos_subscriber rclcpp std_msgs)

add_executable(dynamic_qos_subscriber src/dynamic_qos_subscriber.cpp)
ament_target_dependencies(dynamic_qos_subscriber rclcpp std_msgs)

add_executable(lifecycle_example src/lifecycle_example.cpp)
ament_target_dependencies(lifecycle_example rclcpp std_msgs lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
  custom_qos_publisher
  custom_qos_subscriber
  dynamic_qos_subscriber
  lifecycle_example
  DESTINATION lib/${PROJECT_NAME}
)

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

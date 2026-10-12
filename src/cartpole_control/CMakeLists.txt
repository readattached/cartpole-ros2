cmake_minimum_required(VERSION 3.20)
project(cartpole_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(controller_manager REQUIRED)
find_package(cartpole_description REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(lqr_balancer
  src/main.cpp
  src/lqr_balancer.cpp
)

target_link_libraries(lqr_balancer PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  Eigen3::Eigen
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

install(TARGETS
  lqr_balancer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

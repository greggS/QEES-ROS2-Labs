cmake_minimum_required(VERSION 3.5)
project(test_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_controller test_controller.cpp)
ament_target_dependencies(test_controller rclcpp std_msgs)

install(TARGETS
  test_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

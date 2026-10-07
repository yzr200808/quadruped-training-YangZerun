cmake_minimum_required(VERSION 3.8)
project(mit_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mit_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mit_controller_node src/mit_controller_node.cpp)
target_compile_options(mit_controller_node PRIVATE -O2)
ament_target_dependencies(mit_controller_node
  rclcpp
  std_msgs
  mit_interfaces
  sensor_msgs
  geometry_msgs
)
target_link_libraries(mit_controller_node Threads::Threads)

install(TARGETS
  mit_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过没有版权头/cpplint 的检查（本包尚未声明版权信息）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

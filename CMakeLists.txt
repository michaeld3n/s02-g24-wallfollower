cmake_minimum_required(VERSION 3.8)
project(wall_follower)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(wall_follower_core src/scan_reader.cpp src/controller.cpp)
target_compile_features(wall_follower_core PUBLIC cxx_std_17)
target_include_directories(wall_follower_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_executable(wall_follower_node src/main.cpp src/wall_follower_node.cpp)
target_link_libraries(wall_follower_node wall_follower_core)
ament_target_dependencies(wall_follower_node
  rclcpp rcl_interfaces sensor_msgs geometry_msgs std_msgs std_srvs)

install(TARGETS wall_follower_core EXPORT wall_follower_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS wall_follower_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(wall_follower_targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Official Jazzy tutorial runner assigns a separate ROS domain to this test.
  add_launch_test(test/test_integration.py
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py" TIMEOUT 120)
endif()
ament_package()

cmake_minimum_required(VERSION 3.5)
project(atr_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(atr_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(atr_tracker src/atr_tracker.cpp)
ament_target_dependencies(atr_tracker rclcpp std_msgs atr_interfaces tf2 tf2_ros
tf2_eigen tf2_geometry_msgs visualization_msgs)

add_executable(atr_state_list_listener src/atr_state_list_listener.cpp)
ament_target_dependencies(atr_state_list_listener rclcpp std_msgs atr_interfaces tf2 tf2_ros
tf2_eigen tf2_geometry_msgs visualization_msgs)

install(TARGETS atr_tracker atr_state_list_listener
  DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake rclcpp std_msgs atr_interfaces tf2 tf2_ros
tf2_eigen tf2_geometry_msgs visualization_msgs)


ament_package()
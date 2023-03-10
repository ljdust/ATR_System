cmake_minimum_required(VERSION 3.5)
project(atr_interfaces)

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

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(srv_files
"srv/ConnectionSrv.srv"
"srv/WakeupSrv.srv"
"srv/ChangeWatchdogSrv.srv"
"srv/StartMeasurementSrv.srv"
"srv/ChangeListPublisherSrv.srv"
)

set(msg_files
"msg/ObjectClass.msg"
"msg/ObjectStamped.msg"
"msg/ObjectType.msg"
"msg/ATRState.msg"
"msg/ATRStateStamped.msg"
"msg/ATRStateList.msg"
"msg/ATRStateListStamped.msg"
"msg/ATRPoseData.msg"
"msg/ATRPoseType.msg"
"msg/ATRStateOverall.msg"
"msg/ATRBatteryState.msg"
"msg/ATRCollisionSensor.msg"
"msg/ATRCollisionState.msg"
"msg/ATRMotionData.msg"
"msg/ATRStateActuator.msg"
"msg/ATRStateLoad.msg"
"msg/ATRStateMission.msg"
"msg/ATRStateSignals.msg"
"msg/TimeStampRobot.msg"
"msg/ATRTimeStamps.msg"
"msg/ATRTimeStampsArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)


ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

ament_package()

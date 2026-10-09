cmake_minimum_required(VERSION 3.8)
project(limo_wf_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WallInfo.msg"
  DEPENDENCIES std_msgs
)

ament_package()
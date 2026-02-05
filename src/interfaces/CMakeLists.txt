cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/PackageState.msg"
  "msg/WorldSnapshot.msg"
  "msg/WorldEvent.msg"
  "srv/SetPackageInfo.srv"
  "srv/GetWorldSnapshot.srv"
  "srv/ResetEpisode.srv"
  "srv/Enqueue.srv"
  "srv/Dequeue.srv"
  "action/Pick.action"
  "action/Charge.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
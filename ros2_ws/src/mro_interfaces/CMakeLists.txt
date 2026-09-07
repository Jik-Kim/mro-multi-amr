cmake_minimum_required(VERSION 3.8)
project(mro_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorkerDescriptor.msg"
  "msg/Mission.msg"
  "msg/FleetStatus.msg"
  "msg/TargetHint.msg"
  "msg/Intervention.msg"
  "srv/RequestMission.srv"
  "srv/ZoneToken.srv"
  "srv/OperatorCommand.srv"
  "srv/ManualPair.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

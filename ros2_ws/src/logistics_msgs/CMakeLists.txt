cmake_minimum_required(VERSION 3.8)
project(logistics_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskStep.msg"
  "srv/OptimizeRoute.srv"
  "srv/DispatchGoal.srv"
  "srv/ReportGoalEvent.srv"
  "srv/LookupGoal.srv"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

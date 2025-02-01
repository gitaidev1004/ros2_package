cmake_minimum_required(VERSION 3.8)
project(my_action1)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ✅ 액션 메시지 빌드 설정
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Countdown.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
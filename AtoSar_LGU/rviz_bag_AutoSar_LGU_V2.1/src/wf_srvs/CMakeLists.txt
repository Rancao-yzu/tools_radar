cmake_minimum_required(VERSION 3.0.2)
project(wf_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation  # 添加此项，用于生成自定义消息和服务
)

add_service_files(
  FILES
  PlaySingleFrame.srv  # 添加自定义的服务文件
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

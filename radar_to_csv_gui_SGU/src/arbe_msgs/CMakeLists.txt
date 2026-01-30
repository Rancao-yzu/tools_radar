cmake_minimum_required(VERSION 2.8.3)  # ROS 1 最低要求版本
project(arbe_msgs)

# 设置 C 和 C++ 标准（可选，和 ROS 2 保持一致）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

# 声明消息文件
add_message_files(
  FILES
  wfSguRawData.msg
  wfSgu.msg
)

# 生成消息，指定依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 定义 catkin 包
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)
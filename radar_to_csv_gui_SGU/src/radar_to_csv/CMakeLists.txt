cmake_minimum_required(VERSION 3.0.2)
project(radar_to_csv)   # 和 package.xml的<name> 一致
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS相关包
find_package(catkin REQUIRED COMPONENTS
  roscpp      # ROS C++客户端库
  rosbag      # ROS bag文件操作
  arbe_msgs   # 自定义雷达消息类型
)

# 查找Qt5库
find_package(Qt5 COMPONENTS Core Widgets Concurrent REQUIRED)

# Catkin包配置
catkin_package(
  CATKIN_DEPENDS roscpp rosbag arbe_msgs  # 声明此包依赖的其他ROS包
)

# 包含目录
include_directories(
  include                           # 自定义头文件目录
  ${catkin_INCLUDE_DIRS}            # ROS包含目录
  ${Qt5Widgets_INCLUDE_DIRS}        # Qt5包含目录
  ${Qt5Concurrent_INCLUDE_DIRS}
)

# 设置moc
set(CMAKE_AUTOMOC ON)    # 自动处理Qt的moc（元对象编译器）
set(CMAKE_AUTORCC ON)    # 自动处理Qt资源文件
set(CMAKE_AUTOUIC ON)    # 自动处理Qt界面文件

# 添加可执行文件
add_executable(radar_to_csv_node    # 生成的可执行文件名
  src/radar_to_csv.cpp              # 源文件
)

# 链接库
target_link_libraries(radar_to_csv_node
  ${catkin_LIBRARIES}    # ROS库
  Qt5::Widgets           # Qt界面库
  Qt5::Concurrent        # Qt并发库
  Qt5::Core              # Qt核心库
)

# 添加依赖
add_dependencies(radar_to_csv_node
  ${catkin_EXPORTED_TARGETS}
)

# 安装目标
install(TARGETS radar_to_csv_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




include(CPack)

set(CPACK_PACKAGE_NAME "radar_to_csv_node")
set(CPACK_PACKAGE_VERSION "2.1.0")
set(CPACK_PACKAGE_DESCRIPTION "GUI Tool for extracting radar objects from ROS bag to CSV")
set(CPACK_PACKAGE_CONTACT "you@example.com")

# 支持生成 .tar.gz 和 .zip 压缩包
set(CPACK_GENERATOR "TGZ;ZIP")

# 设置可执行程序
install(TARGETS radar_to_csv_node
  RUNTIME DESTINATION bin
)

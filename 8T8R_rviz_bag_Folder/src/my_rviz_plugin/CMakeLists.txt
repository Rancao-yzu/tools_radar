cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  rosbag
  sensor_msgs
  arbe_msgs
  wf_srvs
)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
qt5_wrap_cpp(MOC_FILES
  include/my_rviz_plugin/bag_reader.h
  include/my_rviz_plugin/my_rviz_plugin.h
)
set(SOURCE_FILES
  src/bag_reader.cpp 
  src/my_rviz_plugin.cpp
  ${MOC_FILES}

)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_rviz_plugin
  CATKIN_DEPENDS roscpp rviz std_msgs rosbag sensor_msgs  arbe_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS})

add_library(my_rviz_plugin ${SOURCE_FILES})
target_link_libraries(my_rviz_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(my_rviz_plugin arbe_msgs_generate_messages_cpp wf_srvs_generate_messages_cpp)

install(TARGETS my_rviz_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)
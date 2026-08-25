#!/bin/bash
# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ROS环境
source /opt/ros/noetic/setup.bash

# 设置环境变量
export LD_LIBRARY_PATH=$SCRIPT_DIR/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$SCRIPT_DIR:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=$SCRIPT_DIR:$CMAKE_PREFIX_PATH

# 检查插件是否被识别
echo "插件注册:"
rospack plugins --attrib=plugin rviz 2>/dev/null

# 启动RViz
echo "正在启动 AutoSar Radar RViz 插件..."
rviz -d $SCRIPT_DIR/config.rviz

#!/bin/bash
# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "请先source ROS环境："
    echo "source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 设置环境变量
export LD_LIBRARY_PATH=$SCRIPT_DIR/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$SCRIPT_DIR:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=$SCRIPT_DIR:$CMAKE_PREFIX_PATH

# 验证插件描述文件
if [ ! -f "$SCRIPT_DIR/plugin_description.xml" ]; then
    echo "错误：找不到插件描述文件"
    exit 1
fi

# 验证库文件
if [ ! -f "$SCRIPT_DIR/lib/libmy_rviz_plugin.so" ]; then
    echo "错误：找不到插件库文件"
    exit 1
fi

# 检查插件是否被识别
echo "检查插件注册..."
rospack plugins --attrib=plugin rviz 2>/dev/null | grep my_rviz_plugin

# 启动RViz
echo "正在启动 AutoSar Radar RViz 插件..."
rviz -d $SCRIPT_DIR/config.rviz

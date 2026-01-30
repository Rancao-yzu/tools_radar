#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "脚本目录: $SCRIPT_DIR"

# 设置环境
source devel/setup.bash

# 前台运行 roslaunch
#roslaunch my_rviz_plugin player.launch

# 使用完整路径启动
roslaunch "$SCRIPT_DIR/src/my_rviz_plugin/launch/player.launch"

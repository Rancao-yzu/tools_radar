#!/bin/bash
set -e
cd "$(dirname "$0")"
source /opt/ros/noetic/setup.bash

# 首次运行或消息定义（.msg/.srv）更新后自动重建
NEED_BUILD=0
if [ ! -d devel/lib/python3/dist-packages/arbe_msgs ]; then
  NEED_BUILD=1
elif [ -n "$(find src -name '*.msg' -o -name '*.srv' -newer devel/lib/python3/dist-packages/arbe_msgs -print -quit)" ]; then
  NEED_BUILD=1
fi
if [ "$NEED_BUILD" -eq 1 ]; then
  # 系统 cmake 版本较新，需要兼容旧策略
  catkin_make --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
fi

# 没有运行的 roscore 时自动启动
if ! rostopic list >/dev/null 2>&1; then
  roscore >/dev/null 2>&1 &
  sleep 2
fi

python3 my_rviz_player.py

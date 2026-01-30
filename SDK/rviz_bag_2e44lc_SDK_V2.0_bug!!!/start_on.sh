#!/bin/bash

source devel/setup.bash


if ! command -v wmctrl &> /dev/null; then
    echo "错误:wmctrl 未安装。请先运行:sudo apt install wmctrl"
    exit 1
fi

# 后台函数：检测 RViz 窗口并尝试置顶
try_set_rviz_on_top() {
  max_tries=10
  try=0
  while [ $try -lt $max_tries ]; do
    if wmctrl -l | grep -i "config.rviz.* - RViz" > /dev/null; then
      wmctrl -l | grep -i "config.rviz.* - RViz" | head -n 1 | awk '{print $1}' | xargs -I {} wmctrl -i -r {} -b add,above
      echo "找到 RViz 窗口并设置为置顶"
      exit 0
    fi
    sleep 2
    try=$((try + 1))
  done
  echo "未能找到 RViz 窗口: config.rviz* - RViz"
  exit 1
}

# 后台运行窗口检测
try_set_rviz_on_top &

# 前台运行 roslaunch
roslaunch my_rviz_plugin player.launch
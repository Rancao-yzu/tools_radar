#!/bin/bash

OUR_ROSCORE_PID=""

is_roscore_running() {  # 检查是否有 roscore 进程在运行
    pgrep -x "roscore" > /dev/null
    return $?
}

if ! is_roscore_running; then
    echo "[INFO] roscore 未运行，正在后台启动 roscore..."
    roscore &
    OUR_ROSCORE_PID=$!
    echo "[INFO] roscore 已启动,PID = $OUR_ROSCORE_PID"

    sleep 1
else
    echo "[INFO] roscore 已经在运行，无需启动。"
fi


source devel/setup.bash
rosrun radar_to_csv radar_to_csv_node _bag_folder:=/home/zjh/下载 _vehicle_id:=2025RW _output_folder:=GT



if [ -n "$OUR_ROSCORE_PID" ]; then
    if ps -p $OUR_ROSCORE_PID > /dev/null; then
        echo "[INFO] 正在关闭启动的 roscore (PID=$OUR_ROSCORE_PID)..."
        kill $OUR_ROSCORE_PID
    else
        echo "[INFO] 我们启动的 roscore (PID=$OUR_ROSCORE_PID) 已经退出。"
    fi
else
    echo "[INFO] 没有需要关闭的 roscore"
fi

echo "[INFO] 脚本执行完毕。"


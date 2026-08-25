#!/bin/bash
# RViz 插件启动脚本：选择 .so 版本后软链为 libmy_rviz_plugin.so 再启动 rviz

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$SCRIPT_DIR/lib"
TARGET="$LIB_DIR/libmy_rviz_plugin.so"

# 收集所有 .so 文件（仅文件名，排序）
mapfile -t SO_FILES < <(find "$LIB_DIR" -maxdepth 1 -name '*.so' -printf '%f\n' 2>/dev/null | sort)
if [ ${#SO_FILES[@]} -eq 0 ]; then
    echo "错误：$LIB_DIR 下没有 .so 文件" >&2
    exit 1
fi

# 显示菜单
echo "请选择插件版本（详见 Readme.md）："
for i in "${!SO_FILES[@]}"; do
    printf "  [%d] %s\n" $((i+1)) "${SO_FILES[i]}"
done
echo "  [0] 退出"

# 读取选择
read -rp "请输入编号: " choice
if [ "$choice" = "0" ]; then
    exit 0
elif [[ ! "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#SO_FILES[@]} )); then
    echo "无效的选择: $choice" >&2
    exit 1
fi

SELECTED="$LIB_DIR/${SO_FILES[choice-1]}"
echo "已选择: $SELECTED"

# 创建符号链接
ln -sf "$SELECTED" "$TARGET"

# 设置 ROS 环境
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH="$LIB_DIR:$LD_LIBRARY_PATH"
export ROS_PACKAGE_PATH="$SCRIPT_DIR:$ROS_PACKAGE_PATH"
export CMAKE_PREFIX_PATH="$SCRIPT_DIR:$CMAKE_PREFIX_PATH"

echo "插件注册:"
rospack plugins --attrib=plugin rviz 2>/dev/null
echo "正在启动 AutoSar Radar RViz 插件..."
rviz -d "$SCRIPT_DIR/config.rviz"
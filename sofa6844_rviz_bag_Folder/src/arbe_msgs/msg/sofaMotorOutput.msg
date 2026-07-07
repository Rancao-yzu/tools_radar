# sofaMotorOutput.msg — 电机状态输出 (LIN 采集后发布)
# 发布 topic: /wf/motor/motor_pub

std_msgs/Header header

# ---- 电机状态 ----
uint8 learn_status      # 自学习状态: 0=未完成, 1=已完成
uint16 position         # 当前位置 (原始值)
uint16 max_stroke       # 最大行程 (原始值)
uint8 fault             # 故障码

# ---- 冗余/扩展字段 ----
float32 position_mm     # 位置 (mm, 如有换算)
float32 stroke_mm       # 行程 (mm, 如有换算)
bool connected          # 设备是否已连接
bool polling            # 是否正在轮询
float32 sample_period   # 采样周期 (秒)
uint8 raw_data0         # 响应帧 DATA[0] 原始值
uint8 raw_data1         # 响应帧 DATA[1] 原始值
string learn_text       # 自学习状态文本
string fault_text       # 故障描述

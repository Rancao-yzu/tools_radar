# 角雷达ADAS报警KPI评估工具 - 使用说明
## eval_warning_metrics_cn_2026
## 概述
本工具用于评估角雷达ADAS报警系统的性能指标，按照"博世角雷达真值系统功能验证评价标准"实现。

通过对比真值系统(GT)和被测系统(WF)的报警数据，计算TP、FP、FN、迟报、早报等多项KPI指标。

## 系统要求
- Python 3.6+
- 依赖包：
  ```bash
  pip install rosbag openpyxl
  ```
- ROS1环境（用于解析bag文件）

## 快速开始

### 1. 准备数据
- 确保当前目录下有rosbag文件（.bag格式）
- bag文件应包含以下topic：
  - 真值：`/corner_radar/sil/warning_status`
  - 被测：`/corner_radar/warning_status`

### 2. 基本使用方法
```bash
# 1. 左前雷达功能评估
python3 eval_warning_metrics_cn_2026.py --radar-id 1 --radar-id-wf 2

# 2. 右前雷达功能评估
python3 eval_warning_metrics_cn_2026.py --radar-id 2 --radar-id-wf 2

# 3. 左后雷达功能评估
python3 eval_warning_metrics_cn_2026.py --radar-id 3 --radar-id-wf 4

# 4. 右后雷达功能评估
python3 eval_warning_metrics_cn_2026.py --radar-id 4 --radar-id-wf 4
```

### 3. 参数说明
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--radar-id` | **GT侧雷达ID** |必须指定|
| `--radar-id-wf` | **WF侧雷达ID** | 4 |
| `--frame-tol` | 帧号容差 | 15 |
| `--merge-gap-frames` | GT端防抖合并的最大间隙帧数 | 2 |


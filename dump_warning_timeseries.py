#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dump_warning_timeseries.py

功能:
- 从当前目录下的所有 *.bag 中,分别把“真值GT”和“被测WF”的报警位解析为按列展开的CSV。
- 每个bag会输出两个CSV:
    1) <bag_name>_GT_timeseries.csv
    2) <bag_name>_WF_timeseries.csv
- 每个CSV的列为:
    timestamp, frame_idx, radar_id, LeftBsd, RightBsd, LeftLca, RightLca, LeftDow, RightDow,
    Rcw, LeftRcta, RightRcta, LeftRctb, RightRctb, LeftFcta, RightFcta, LeftFctb, RightFctb

使用方式(在有ROS环境的机器上):
    python3 dump_warning_timeseries.py --gt /corner_radar/sil/warning_status --wf /corner_radar/warning_status --radar-id 3
说明:
- --radar-id 仅用于过滤GT/WF消息中的 data[0](radar_id),不指定则不过滤。
- 不进行RCW左右合并;如果你需要“RCW在GT端左右合并”的事件级比较,请继续使用你已有的 eval_warning_metrics_cn_1010.py。
"""
import os
import csv
import argparse

import rosbag
from std_msgs.msg import Int32MultiArray

FUNC_NAMES = [
    None,
    "LeftBsd","RightBsd",
    "LeftLca","RightLca",
    "LeftDow","RightDow",
    "Rcw",
    "LeftRcta","RightRcta",
    "LeftRctb","RightRctb",
    "LeftFcta","RightFcta",
    "LeftFctb","RightFctb",
]

def write_timeseries_csv(bag_path, topic, radar_id_filter, out_csv):
    fields = ["timestamp","frame_idx","radar_id"] + [FUNC_NAMES[i] for i in range(1,16)]
    frame_idx = -1
    rows = []

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
            if not data or len(data) < 16:
                continue
            rid = int(data[0])
            if radar_id_filter is not None and rid != radar_id_filter:
                continue
            frame_idx += 1
            ts = t.to_sec()
            row = {
                "timestamp": f"{ts:.9f}",
                "frame_idx": frame_idx,
                "radar_id": rid,
            }
            for i in range(1,16):
                row[FUNC_NAMES[i]] = 1 if int(data[i]) != 0 else 0
            rows.append(row)

    if not rows:
        # 即使没有数据也写一个空表头,便于后续读取
        with open(out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
        return

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

def main():
    parser = argparse.ArgumentParser(description="将角雷达GT/WF报警位拆解为时序列CSV(逐列展开)。")
    parser.add_argument("--gt", default="/corner_radar/sil/warning_status", help="真值GT topic")
    parser.add_argument("--wf", default="/corner_radar/warning_status", help="被测WF topic")
    parser.add_argument("--radar-id", type=int, default=None, help="按 data[0] 过滤的 radar_id(不指定则不过滤)")
    args = parser.parse_args()

    base_dir = os.getcwd()
    bag_files = [os.path.join(base_dir, f) for f in os.listdir(base_dir) if f.lower().endswith(".bag")]
    bag_files.sort()
    if not bag_files:
        print("[WARN] 当前目录没有 .bag 文件")
        return

    for bf in bag_files:
        base = os.path.splitext(os.path.basename(bf))[0]
        gt_csv = os.path.join(base_dir, f"{base}_GT_timeseries.csv")
        wf_csv = os.path.join(base_dir, f"{base}_WF_timeseries.csv")

        print(f"[INFO] 处理 {os.path.basename(bf)} ...")
        write_timeseries_csv(bf, args.gt, args.radar_id, gt_csv)
        write_timeseries_csv(bf, args.wf, args.radar_id, wf_csv)
        print(f"  - 已写出:{gt_csv}")
        print(f"  - 已写出:{wf_csv}")

if __name__ == "__main__":
    main()

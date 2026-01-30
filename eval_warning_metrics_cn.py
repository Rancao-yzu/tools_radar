#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
评估角雷达ADAS报警的KPI指标(按“博世角雷达真值系统功能验证评价标准”实现)
================================================================
python3 eval_warning_metrics_cn_2026.py --radar-id 1 --radar-id-wf 2
左前雷达功能

python3 eval_warning_metrics_cn_2026.py --radar-id 2 --radar-id-wf 2
右前雷达功能

python3 eval_warning_metrics_cn_2026.py --radar-id 3 --radar-id-wf 4
左后雷达功能

python3 eval_warning_metrics_cn_2026.py --radar-id 4 --radar-id-wf 4
右后雷达功能

对于WF端，radar_id_3是从雷达，3的报警从4传出，同理，1的报警从2传出。
对于GT端，报警信息从各雷达直接传出。
================================================================
- RCW(功能索引=7)在“真值GT端”对“左右雷达”进行合并处理:
    只要左右任一雷达报,则视为GT的RCW=1。

数据来源:ROS1 bag,两个topic均为 Int32MultiArray:
- 真值:/corner_radar/sil/warning_status  
- 被测:/corner_radar/warning_status

约定的数据布局:
- data[0] = radar_id(整型)
- data[1..15] = 15个报警功能位(0/1),顺序:
  1: LeftBsd, 2: RightBsd, 3: LeftLca, 4: RightLca, 5: LeftDow, 6: RightDow,
  7: Rcw, 8: LeftRcta, 9: RightRcta, 10: LeftRctb, 11: RightRctb,
  12: LeftFcta, 13: RightFcta, 14: LeftFctb, 15: RightFctb

================================================================
评价核心(按PPT规则的“帧号±15帧”和“区间交叠”实现):
----------------------------------------------------------------
1) 先把每个功能位的二值序列转为“报警区间”(连续为1的帧段)
   - 对GT得到若干 GTWARY(每个区间:起始帧号/结束帧号/起止时间戳)
   - 对WF得到若干 WFWARY(同上)
   - “帧号”定义:以各topic自身消息顺序作为帧序号(0,1,2,...)。

2) 区间交叠判定(时间域交叠):
   - 两个区间 [s1,e1] 与 [s2,e2] 只要有时间交集(max(s1,s2) < min(e1,e2))即认为“交叠”。

3) 计数规则(帧号比较均用“帧序号差”且阈值默认±15帧):
   设“帧差容限”为 FRAME_TOL(默认15)

   A) 针对每个 WFWARY:
      - 若该 WFWARY 与某功能的多个(N≥2)GTWARY交叠:
          (1) 比较 WFWARY_STARTFRM 与“第一个交叠的GTWARY”的 GTWARY_STARTFRM:
              · 若 GTWARY_STARTFRM < WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 迟报 LA += 1
              · 若 GTWARY_STARTFRM > WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 早报 EA += 1
          (2) 比较 WFWARY_ENDFRM 与“最后一个交叠的GTWARY”的 GTWARY_ENDFRM:
              · 若 GTWARY_ENDFRM < WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 迟退 LD += 1
              · 若 GTWARY_ENDFRM > WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 早退 ED += 1
          (3) 认为“存在正报和误报场景”,即 TP += N,FP += N-1

      - 若该 WFWARY 与 0 个 GTWARY 交叠:
          → 误报 FP += 1

   B) 针对每个 GTWARY:
      - 若该 GTWARY 仅与 1 个 WFWARY 交叠:
          (1) 比较各自的起始帧:
              · 若 GTWARY_STARTFRM < WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 迟报 LA += 1
              · 若 GTWARY_STARTFRM > WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 早报 EA += 1
          (2) 比较各自的结束帧:
              · 若 GTWARY_ENDFRM < WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 迟退 LD += 1
              · 若 GTWARY_ENDFRM > WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 早退 ED += 1
          (3) 正报 TP += 1

      - 若该 GTWARY 与多个(N≥2) WFWARY 交叠:
          (1) 与A)相同起止帧比较(首个/最后一个)→ 可能产生LA/EA/LD/ED
          (2) 认为存在“报警间断”,DW += 1

      - 若该 GTWARY 与 0 个 WFWARY 交叠:
          → 漏报 FN += 1

5) KPI 计算:
   - FPR = FP / (FP + FN + TP)
   - FNR = FN / (FN + TP)
   - TPR = TP / (FN + TP)
   - DWR = DW / TP(当TP=0时为NaN)
   - LAR = LA / (FN + TP)
   - EAR = EA / (FN + TP)
   - LDR = LD / (FN + TP)
   - EDR = ED / (FN + TP)

"""

import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

import rosbag
from std_msgs.msg import Int32MultiArray

try:
    import openpyxl
    from openpyxl import Workbook
except Exception:
    openpyxl = None 


# 15个功能名称
FUNC_NAMES = [
    None,      
    "LeftBsd",  # 1: 左盲区监测（Blind Spot Detection）
    "RightBsd", # 2: 右盲区监测
    "LeftLca",  # 3: 左变道辅助（Lane Change Assist）
    "RightLca", # 4: 右变道辅助
    "LeftDow",  # 5: 左开门预警（Door Open Warning）
    "RightDow", # 6: 右开门预警
    "Rcw",      # 7: 后方横穿预警（Rear Cross Warning）
    "LeftRcta", # 8: 左后方横穿报警（Rear Cross Traffic Alert）
    "RightRcta",# 9: 右后方横穿报警
    "LeftRctb", # 10: 左后方横穿制动（Rear Cross Traffic Brake）
    "RightRctb",# 11: 右后方横穿制动
    "LeftFcta", # 12: 左前向横穿报警（Front Cross Traffic Alert）
    "RightFcta",# 13: 右前向横穿报警
    "LeftFctb", # 14: 左前向横穿制动（Front Cross Traffic Brake）
    "RightFctb",# 15: 右前向横穿制动
]

RCW_INDEX = 7  # RCW 功能索引

RIGHT_FUNC_IDXS = {2, 4, 6, 9, 11, 13, 15, 7}   # 右雷达对应的功能索引集合
LEFT_FUNC_IDXS  = {1, 3, 5, 8, 10, 12, 14, 7}

def indices_for_side(radar_id: Optional[int]) -> set:
    """
    根据雷达ID返回对应侧的功能索引集合
    Args:
        radar_id: 雷达ID（None=所有雷达）
    Returns:
        功能索引集合
    """
    if radar_id == 3 or radar_id == 1:
        return LEFT_FUNC_IDXS
    if radar_id == 4 or radar_id == 2:
        return RIGHT_FUNC_IDXS
    return set(range(1, 16))


@dataclass
class Event:
    """单个报警区间(连续为1的帧段)"""
    start_t: float      # 起始时间戳(秒)
    end_t: float        # 结束时间戳(秒)
    start_idx: int      # 起始帧号(本topic内的消息序号)
    end_idx: int        # 结束帧号(本topic内的消息序号)

    def duration(self) -> float:
        """区间持续时长(秒)"""
        return max(0.0, self.end_t - self.start_t) # 确保非负值


@dataclass
class Counts:
    """各类计数器"""
    TP: int = 0
    FP: int = 0
    FN: int = 0
    LA: int = 0  # 迟报
    EA: int = 0  # 早报
    LD: int = 0  # 迟退
    ED: int = 0  # 早退
    DW: int = 0  # 报警间断


def merge_events_by_gap_frames(events: List[Event], max_gap_frames: int) -> List[Event]:
    """
    当 gap0 <= max_gap_frames 时合并为一段 - GT端防抖
    Args:
        events: 事件列表
        max_gap_frames: 允许的最大间隙帧数
    """
    if not events or max_gap_frames <= 0:   # 空列表或最大间隙帧数<=0，直接返回
        return events
    evs = sorted(events, key=lambda e: (e.start_idx, e.start_t))# 按起始帧号和时间排序
    merged: List[Event] = [Event(evs[0].start_t, evs[0].end_t, evs[0].start_idx, evs[0].end_idx)]   # 初始化合并列表
    for e in evs[1:]:   # 遍历后续事件
        last = merged[-1]
        gap0 = e.start_idx - last.end_idx - 1   # 计算两个事件之间的帧间隙
        if gap0 <= max_gap_frames:  # 间隙小于等于允许值，合并事件
            last.end_t   = max(last.end_t, e.end_t) # 更新结束时间为较晚者
            last.end_idx = max(last.end_idx, e.end_idx) # 更新结束帧号为较大者
        else:
            merged.append(Event(e.start_t, e.end_t, e.start_idx, e.end_idx))
    return merged

def safe_div(n: int, d: int) -> float:
    """安全除法,分母为0时返回NaN"""
    return float(n) / float(d) if d > 0 else float("nan")


def read_samples_with_index_GT(bag: rosbag.Bag, topic: str, radar_id: Optional[int],
                               merge_rcw_lr: bool,
                               merge_gap_frames: int = 2) -> Dict[int, List[Tuple[float, int, int]]]:
    """
    读取GTtopic,提取每个功能位的三元组序列：(timestamp, bit, frame_idx)
    
    Args:
        bag: ROS bag文件对象
        topic: GT topic名称
        radar_id: 雷达ID过滤条件
        merge_rcw_lr: 是否合并RCW的左右雷达数据
        merge_gap_frames: GT端事件去抖合并的最大间隙帧数

    """
    # Step1: 建立“按radar_id过滤”的采样(非RCW)
    raw_filtered: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx2 = -1
    if radar_id is None:    # 未指定雷达ID
        # 重新遍历逐帧做 BSD/LCA 融合(不直接拷贝 raw_all)
        for _, msg, t in bag.read_messages(topics=[topic]):
            data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
            if not data or len(data) < 16:
                continue
            frame_idx2 += 1
            ts = t.to_sec()

            # 先取出原始位
            b1 = 1 if int(data[1]) != 0 else 0  # LeftBsd
            b2 = 1 if int(data[2]) != 0 else 0  # RightBsd
            b3 = 1 if int(data[3]) != 0 else 0  # LeftLca
            b4 = 1 if int(data[4]) != 0 else 0  # RightLca

            # 融合:BSD/LCA 互为 OR,并回写到两个功能位
            left_or  = 1 if (b1 or b3) else 0
            right_or = 1 if (b2 or b4) else 0

            # 写入 1..15  字典：键=功能
            raw_filtered[1].append((ts, left_or,  frame_idx2))   # LeftBsd
            raw_filtered[3].append((ts, left_or,  frame_idx2))   # LeftLca
            raw_filtered[2].append((ts, right_or, frame_idx2))   # RightBsd
            raw_filtered[4].append((ts, right_or, frame_idx2))   # RightLca
            for i in (5,6,7,8,9,10,11,12,13,14,15):
                bit = 1 if int(data[i]) != 0 else 0
                raw_filtered[i].append((ts, bit, frame_idx2))
    else:
        # === 修改点2:Step2 - 按指定 radar_id 过滤的情形 ===
        for _, msg, t in bag.read_messages(topics=[topic]):
            data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
            if not data or len(data) < 16:
                continue
            if int(data[0]) != radar_id:
                continue
            frame_idx2 += 1
            ts = t.to_sec()

            b1 = 1 if int(data[1]) != 0 else 0
            b2 = 1 if int(data[2]) != 0 else 0
            b3 = 1 if int(data[3]) != 0 else 0
            b4 = 1 if int(data[4]) != 0 else 0

            left_or  = 1 if (b1 or b3) else 0
            right_or = 1 if (b2 or b4) else 0

            raw_filtered[1].append((ts, left_or,  frame_idx2))
            raw_filtered[3].append((ts, left_or,  frame_idx2))
            raw_filtered[2].append((ts, right_or, frame_idx2))
            raw_filtered[4].append((ts, right_or, frame_idx2))
            for i in (5,6,7,8,9,10,11,12,13,14,15):
                bit = 1 if int(data[i]) != 0 else 0
                raw_filtered[i].append((ts, bit, frame_idx2))

    # 单独处理ID=4 的数据用于RCW
    raw_filtered_id4: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx4 = -1
    for _, msg, t in bag.read_messages(topics=[topic]):
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue
        if int(data[0]) != 4:   # 仅保留右雷达（ID=4）
            continue
        frame_idx4 += 1
        ts = t.to_sec()
        for i in range(1, 16):
            bit = 1 if int(data[i]) != 0 else 0
            raw_filtered_id4[i].append((ts, bit, frame_idx4))

    # Step2: 生成最终GT采样:
    out: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    for i in range(1, 16):
        if i == RCW_INDEX:
            base = sorted(raw_filtered_id4[i], key=lambda x: x[0]) # 使用右雷达数据，按时间排序
        else:
            base = sorted(raw_filtered[i], key=lambda x: x[0])

        # GT端“短间隙合并防抖”
        evs = samples_to_events(base) #包含至少一个完整的报警区间时，events不为空。
        evs_m = merge_events_by_gap_frames(evs, merge_gap_frames) if merge_gap_frames > 0 else evs

        merged_samples = []
        for ev in evs_m:
            merged_samples.append((ev.start_t, 1, ev.start_idx))
            merged_samples.append((ev.end_t, 0, ev.end_idx))
        
        if merged_samples:
            merged_samples.sort(key=lambda x: x[0])
            out[i] = merged_samples
        else:
            out[i] = base

    return out


def read_samples_with_index_WF(bag: rosbag.Bag, topic: str, radar_id: Optional[int]) -> Dict[int, List[Tuple[float, int, int]]]:
    """
    读取WF topic,提取每个功能位的三元组序列:(timestamp, bit, frame_idx)
    - 完全保持当前状态(不做RCW合并)
    - 若指定radar_id,则仅保留匹配的消息;否则不过滤
    """
    out: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx = -1
    for _, msg, t in bag.read_messages(topics=[topic]):
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue
        if radar_id is not None and int(data[0]) != radar_id: # 雷达ID过滤
            continue
        frame_idx += 1
        ts = t.to_sec()
        for i in range(1, 16):
            bit = 1 if int(data[i]) != 0 else 0
            out[i].append((ts, bit, frame_idx))
    for i in range(1, 16):
        out[i].sort(key=lambda x: x[0])
    return out


def samples_to_events(samples: List[Tuple[float, int, int]]) -> List[Event]:
    """
    将某功能位的 (t, bit, idx) 序列转为报警区间（连续为1的段）
    - rising：bit由0->1 的位置作为区间起点
    - falling：bit由1->0 的位置作为区间终点
    - 若末尾仍为1，则以最后一帧的时间与索引收尾
    """
    events: List[Event] = []
    if not samples:
        return events

    prev_t, prev_b, prev_idx = samples[0]  # 前一个采样点
    high_start_t = None  # 高电平（报警）起始时间
    high_start_idx = None  # 高电平起始帧号

    # 初始化：若首帧即为1，则视为在此处上升沿
    if prev_b == 1:
        high_start_t = prev_t
        high_start_idx = prev_idx

    # 遍历后续采样点
    for t, b, idx in samples[1:]:
        if b == prev_b:  # 位状态未变，继续
            prev_t, prev_b, prev_idx = t, b, idx
            continue
        # 发生跳变
        if prev_b == 0 and b == 1:  # 上升沿（0->1）
            high_start_t = t  # 记录起始时间
            high_start_idx = idx  # 记录起始帧号
        elif prev_b == 1 and b == 0:  # 下降沿（1->0）
            # 收集完成的高电平区间
            if high_start_t is not None and high_start_idx is not None:
                events.append(Event(start_t=high_start_t, end_t=t, start_idx=high_start_idx, end_idx=idx))
                high_start_t = None  # 重置起始点
                high_start_idx = None
        prev_t, prev_b, prev_idx = t, b, idx  # 更新前一个采样点

    # 收尾：若最后仍为高电平，则用末帧收尾
    last_t, last_b, last_idx = samples[-1]
    if last_b == 1 and high_start_t is not None and high_start_idx is not None:
        events.append(Event(start_t=high_start_t, end_t=last_t, start_idx=high_start_idx, end_idx=last_idx))
    return events


def intervals_overlap(a: Event, b: Event) -> bool:
    """时间域交叠判定：只要有时间交集即为True"""
    return max(a.start_t, b.start_t) < min(a.end_t, b.end_t)


def evaluate_one_function(gt_samps: List[Tuple[float, int, int]],
                          wf_samps: List[Tuple[float, int, int]],
                          frame_tol: int) -> Tuple[Counts, int, int]:
    """
    - gt_samps / wf_samps:对应功能位的 (t, bit, idx) 序列
    - frame_tol:帧号容差(±frame_tol),默认15
    - 返回:Counts, gt_event_count, wf_event_count
    """
    c = Counts()  # 初始化计数器
    gt_events = samples_to_events(gt_samps)  # GT事件列表
    wf_events = samples_to_events(wf_samps)  # WF事件列表
    tol_sec = frame_tol * 0.066  # 将帧容差转换为时间容差（假设每帧0.066秒）

    # 预计算交叠关系
    gt_overlaps: Dict[int, List[int]] = {i: [] for i in range(len(gt_events))}  # 每个GT区间交叠的WF区间下标
    wf_overlaps: Dict[int, List[int]] = {j: [] for j in range(len(wf_events))}  # 每个WF区间交叠的GT区间下标

    for i, g in enumerate(gt_events):# 遍历所有GT事件
        for j, w in enumerate(wf_events):
            if intervals_overlap(g, w):
                gt_overlaps[i].append(j)
                wf_overlaps[j].append(i)

  # A) 针对每个WF事件（被测系统报警）的规则
    for j, w in enumerate(wf_events):
        ovl_gt = wf_overlaps[j]  # 与该WF事件重叠的GT事件索引列表
        if len(ovl_gt) == 0:  # 无重叠：误报
            c.FP += 1
            continue
        if len(ovl_gt) >= 2:  # 与多个GT事件重叠
            first_g = gt_events[min(ovl_gt)]  # 重叠的最早GT事件
            last_g  = gt_events[max(ovl_gt)]  # 重叠的最晚GT事件

            # 起始帧比较（迟报/早报）
            start_diff_sec = w.start_t - first_g.start_t  # 起始时间差
            if first_g.start_t < w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1  # 迟报：GT开始早于WF，且时间差超过容差
            if first_g.start_t > w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1  # 早报：GT开始晚于WF，且时间差超过容差

            # 结束帧比较（迟退/早退）
            end_diff_sec = w.end_t - last_g.end_t  # 结束时间差
            if last_g.end_t < w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1  # 迟退：GT结束早于WF，且时间差超过容差
            if last_g.end_t > w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1  # 早退：GT结束晚于WF，且时间差超过容差

            # 存在正报和误报场景（按PPT的"WF与多个GT交叠"解释）
            c.TP += len(ovl_gt)  # 每个重叠的GT事件都算一个正报
            c.FP += (len(ovl_gt) - 1)  # 多余的WF事件算误报

    # B) 针对每个GT事件（真值报警）的规则
    for i, g in enumerate(gt_events):
        ovl_wf = gt_overlaps[i]  # 与该GT事件重叠的WF事件索引列表
        if len(ovl_wf) == 0:  # 无重叠：漏报
            c.FN += 1
            continue
        if len(ovl_wf) == 1:  # 与一个WF事件重叠
            w = wf_events[ovl_wf[0]]  # 重叠的WF事件

            # 起始帧比较
            start_diff_sec = w.start_t - g.start_t
            if g.start_t < w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1  # 迟报
            if g.start_t > w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1  # 早报

            # 结束帧比较
            end_diff_sec = w.end_t - g.end_t
            if g.end_t < w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1  # 迟退
            if g.end_t > w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1  # 早退

            c.TP += 1  # 正报

        if len(ovl_wf) >= 2:  # 与多个WF事件重叠：说明报警被间断
            first_w = wf_events[min(ovl_wf)]  # 重叠的最早WF事件
            last_w  = wf_events[max(ovl_wf)]  # 重叠的最晚WF事件

            # 起始帧比较
            start_diff_sec = first_w.start_t - g.start_t
            if g.start_t < first_w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1  # 迟报
            if g.start_t > first_w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1  # 早报

            # 结束帧比较
            end_diff_sec = last_w.end_t - g.end_t
            if g.end_t < last_w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1  # 迟退
            if g.end_t > last_w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1  # 早退

            c.TP += 1  # 正报
            c.DW += 1  # 报警间断（一个GT报警被分成多个WF报警）

    return c, len(gt_events), len(wf_events)


def write_xlsx(out_path: str,
               summary_rows: List[Dict[str, object]],
               by_bag_rows: List[Dict[str, object]]) -> None:
    """写Excel（两个sheet）。若没有openpyxl，则跳过。"""
    if openpyxl is None:
        return
    wb = Workbook()
    # summary sheet
    ws1 = wb.active
    ws1.title = "summary"
    if summary_rows:
        headers = list(summary_rows[0].keys())
        ws1.append(headers)
        for r in summary_rows:
            ws1.append([r.get(h, "") for h in headers])

    # by_bag sheet
    ws2 = wb.create_sheet("by_bag")
    if by_bag_rows:
        headers2 = list(by_bag_rows[0].keys())
        ws2.append(headers2)
        for r in by_bag_rows:
            ws2.append([r.get(h, "") for h in headers2])

    wb.save(out_path)


def evaluate_one_bag(bag_path: str,
                     gt_topic: str,
                     wf_topic: str,
                     radar_id_gt: Optional[int],
                     radar_id_wf: Optional[int],
                     frame_tol: int,
                     merge_gap_frames: int,
                     merge_rcw_lr: bool) -> Tuple[List[Dict[str, object]], Counts]:
    """
    评估单个bag,返回:
    - rows:每个功能一行 + OVERALL(同CSV结构)
    - total_counts:该bag自身的 OVERALL 计数(仅计TP/FP/FN/LA/EA/LD/ED/DW)
    """
    bag = rosbag.Bag(bag_path, "r")

    # GT:带RCW左右合并(仅RCW),并做短间隙合并防抖
    gt_samples_all = read_samples_with_index_GT(bag, gt_topic, radar_id_gt,
                                                merge_rcw_lr=merge_rcw_lr,
                                                merge_gap_frames=merge_gap_frames)

    # WF:保持当前实现(不合并RCW)
    wf_samples_all = read_samples_with_index_WF(bag, wf_topic, radar_id_wf)

    rows: List[Dict[str, object]] = []  # 评估结果行
    total = Counts()  # 总计数器
    total_gt_events = 0  # 总GT事件数
    total_wf_events = 0  # 总WF事件数

    valid_idxs = indices_for_side(radar_id_gt)

    # 逐功能评估
    for idx in sorted(valid_idxs):
        name = FUNC_NAMES[idx] or f"Func{idx}"
        gt_samps = gt_samples_all[idx]
        wf_samps = wf_samples_all[idx]

        c, gt_ev_cnt, wf_ev_cnt = evaluate_one_function(gt_samps, wf_samps, frame_tol=frame_tol)

        denom_all = c.FP + c.FN + c.TP
        denom_pos = c.FN + c.TP

        rows.append({
            "function": name,  # 功能名称
            "TP": c.TP,  # 正报数
            "FP": c.FP,  # 误报数
            "FN": c.FN,  # 漏报数
            "LA": c.LA,  # 迟报数
            "EA": c.EA,  # 早报数
            "LD": c.LD,  # 迟退数
            "ED": c.ED,  # 早退数
            "DW": c.DW,  # 间断数
            "FPR": f"{safe_div(c.FP, denom_all):.6f}",  # 误报率
            "FNR": f"{safe_div(c.FN, denom_pos):.6f}",  # 漏报率
            "TPR": f"{safe_div(c.TP, denom_pos):.6f}",  # 正报率
            "DWR": f"{safe_div(c.DW, c.TP):.6f}" if c.TP > 0 else "nan",  # 间断率
            "LAR": f"{safe_div(c.LA, denom_pos):.6f}",  # 迟报率
            "EAR": f"{safe_div(c.EA, denom_pos):.6f}",  # 早报率
            "LDR": f"{safe_div(c.LD, denom_pos):.6f}",  # 迟退率
            "EDR": f"{safe_div(c.ED, denom_pos):.6f}",  # 早退率
            "gt_events": gt_ev_cnt,  # GT事件数
            "sys_events": wf_ev_cnt,  # 系统事件数
        })


        # 汇总
        total.TP += c.TP
        total.FP += c.FP
        total.FN += c.FN
        total.LA += c.LA
        total.EA += c.EA
        total.LD += c.LD
        total.ED += c.ED
        total.DW += c.DW
        total_gt_events += gt_ev_cnt
        total_wf_events += wf_ev_cnt

    # OVERALL
    denom_all = total.FP + total.FN + total.TP
    denom_pos = total.FN + total.TP
    rows.append({
        "function": "OVERALL",
        "TP": total.TP,
        "FP": total.FP,
        "FN": total.FN,
        "LA": total.LA,
        "EA": total.EA,
        "LD": total.LD,
        "ED": total.ED,
        "DW": total.DW,
        "FPR": f"{safe_div(total.FP, denom_all):.6f}",
        "FNR": f"{safe_div(total.FN, denom_pos):.6f}",
        "TPR": f"{safe_div(total.TP, denom_pos):.6f}",
        "DWR": f"{safe_div(total.DW, total.TP):.6f}" if total.TP > 0 else "nan",
        "LAR": f"{safe_div(total.LA, denom_pos):.6f}",
        "EAR": f"{safe_div(total.EA, denom_pos):.6f}",
        "LDR": f"{safe_div(total.LD, denom_pos):.6f}",
        "EDR": f"{safe_div(total.ED, denom_pos):.6f}",
        "gt_events": total_gt_events,
        "sys_events": total_wf_events,
    })

    return rows, total


def main():
    parser = argparse.ArgumentParser(description="角雷达报警KPI评估(支持目录批量与RCW左右合并)。")
    parser.add_argument("--bag-dir", type=str, default=".",help="包含rosbag文件的目录路径(默认当前目录)")
    parser.add_argument("--gt", default="/corner_radar/sil/warning_status", help="真值topic")
    parser.add_argument("--sys", default="/corner_radar/warning_status", help="被测topic")
    parser.add_argument("--radar-id", type=int, default=None, help="GT侧:除RCW外的功能,若指定则仅评估该radar_id(RCW始终做左右合并)")
    parser.add_argument("--radar-id-wf", type=int, default=4, help="WF侧:被测topic过滤的radar_id(默认4)")
    parser.add_argument("--frame-tol", type=int, default=15, help="帧号容差(±frame-tol),默认15")
    parser.add_argument("--merge-gap-frames", type=int, default=2, help="GT端事件去抖合并:相邻两段之间的“0”间隙帧数 ≤ 该值时合并(默认2;设为0则不合并)")
    #parser.add_argument("--all-in-dir", action="store_true", help="扫描bag所在目录下的所有*.bag并合并统计")
    parser.add_argument("--out", default=None, help="输出CSV路径(默认:单bag为 *_metrics.csv,目录合并为 *_ALL_metrics.csv)")
    parser.add_argument("--xlsx", default=None, help="目录合并时的Excel输出路径(默认:*_ALL_metrics.xlsx)")
    args = parser.parse_args()

    
    base_dir = os.getcwd()

    # 目录批量:搜集所有*.bag
    bag_files = [os.path.join(args.bag_dir, f) for f in os.listdir(args.bag_dir) 
                 if f.lower().endswith(".bag")]
    bag_files.sort()

    if not bag_files:
        print(f"[WARN] 目录 {args.bag_dir} 未发现任何 .bag 文件。")
        return

    print(f"[INFO] 扫描到 {len(bag_files)} 个bag:")
    for bf in bag_files:
        print("   -", os.path.basename(bf))

    # 聚合：逐包评估，累加到“全局功能行”
    agg_rows: Dict[str, Dict[str, object]] = {}  # key=function name
    by_bag_rows: List[Dict[str, object]] = []
    # 初始化功能集合
    for idx in range(1, 16):
        name = FUNC_NAMES[idx] or f"Func{idx}"
        agg_rows[name] = {
            "function": name, "TP": 0, "FP": 0, "FN": 0, "LA": 0, "EA": 0, "LD": 0, "ED": 0,"DW": 0,
            "FPR": "nan", "FNR": "nan", "TPR": "nan", "DWR": "nan", "LAR": "nan", "EAR": "nan", "LDR": "nan", "EDR": "nan",
            "gt_events": 0, "sys_events": 0
        }

    total_all = Counts()
    total_gt_events_all = 0
    total_wf_events_all = 0

    for bf in bag_files:
        rows, total = evaluate_one_bag(
            bag_path=bf,
            gt_topic=args.gt,
            wf_topic=args.sys,
            radar_id_gt=args.radar_id,
            radar_id_wf=args.radar_id_wf,
            frame_tol=args.frame_tol,
            merge_gap_frames=args.merge_gap_frames,
            merge_rcw_lr=True
        )
        # rows 包含每个功能 + OVERALL
        perbag_overall = None
        for r in rows:
            if r["function"] != "OVERALL":
                name = r["function"]
                # 累加功能级计数
                for k in ["TP","FP","FN","LA","EA","LD","ED","DW","gt_events","sys_events"]:
                    agg_rows[name][k] += int(r[k])
            else:
                perbag_overall = r

        # 记录 by_bag 行（按OVERALL）
        if perbag_overall:
            tp=int(perbag_overall["TP"]); dw=int(perbag_overall["DW"])
            by_bag_rows.append({
                "bag": os.path.basename(bf),
                "TP": int(perbag_overall["TP"]),
                "FP": int(perbag_overall["FP"]),
                "FN": int(perbag_overall["FN"]),
                "LA": int(perbag_overall["LA"]),
                "EA": int(perbag_overall["EA"]),
                "LD": int(perbag_overall["LD"]),
                "ED": int(perbag_overall["ED"]),
                "DW": dw,
            })

        # 汇总全局总计
        total_all.TP += total.TP
        total_all.FP += total.FP
        total_all.FN += total.FN
        total_all.LA += total.LA
        total_all.EA += total.EA
        total_all.LD += total.LD
        total_all.ED += total.ED
        total_all.DW += total.DW

        # 统计事件数总和（从rows中的gt_events/sys_events求和）
        total_gt_events_all += sum(int(r["gt_events"]) for r in rows if r["function"] != "OVERALL")
        total_wf_events_all += sum(int(r["sys_events"]) for r in rows if r["function"] != "OVERALL")

    # 计算聚合后的各功能比率
    summary_rows: List[Dict[str, object]] = []
    for idx in range(1, 16):
        name = FUNC_NAMES[idx] or f"Func{idx}"
        r = agg_rows[name]
        denom_all = r["FP"] + r["FN"] + r["TP"]
        denom_pos = r["FN"] + r["TP"]
        r.update({
            "FPR": f"{safe_div(r['FP'], denom_all):.6f}",
            "FNR": f"{safe_div(r['FN'], denom_pos):.6f}",
            "TPR": f"{safe_div(r['TP'], denom_pos):.6f}",
            "DWR": f"{safe_div(r['DW'], r['TP']):.6f}",
            "LAR": f"{safe_div(r['LA'], denom_pos):.6f}",
            "EAR": f"{safe_div(r['EA'], denom_pos):.6f}",
            "LDR": f"{safe_div(r['LD'], denom_pos):.6f}",
            "EDR": f"{safe_div(r['ED'], denom_pos):.6f}",
        })
        summary_rows.append(r)

    # OVERALL（所有功能合计）
    denom_all = total_all.FP + total_all.FN + total_all.TP
    denom_pos = total_all.FN + total_all.TP
    summary_rows.append({
        "function": "OVERALL",
        "TP": total_all.TP,
        "FP": total_all.FP,
        "FN": total_all.FN,
        "LA": total_all.LA,
        "EA": total_all.EA,
        "LD": total_all.LD,
        "ED": total_all.ED,
        "DW": total_all.DW,
        "FPR": f"{safe_div(total_all.FP, denom_all):.6f}",
        "FNR": f"{safe_div(total_all.FN, denom_pos):.6f}",
        "TPR": f"{safe_div(total_all.TP, denom_pos):.6f}",
        "DWR": f"{safe_div(total_all.DW, total_all.TP):.6f}" if total_all.TP > 0 else "nan",
        "LAR": f"{safe_div(total_all.LA, denom_pos):.6f}",
        "EAR": f"{safe_div(total_all.EA, denom_pos):.6f}",
        "LDR": f"{safe_div(total_all.LD, denom_pos):.6f}",
        "EDR": f"{safe_div(total_all.ED, denom_pos):.6f}",
        "gt_events": total_gt_events_all,
        "sys_events": total_wf_events_all,
    })

    # 写Excel(两个sheet)
    excel_name = f"./OUT/radar_{args.radar_id}.xlsx"

    out_xlsx = os.path.join(base_dir, excel_name)
    write_xlsx(out_xlsx, summary_rows, by_bag_rows)
    if openpyxl is not None:
        print(f"已输出Excel：{out_xlsx} （summary + by_bag）")
    else:
        print("[WARN] 当前环境未安装 openpyxl, pip install openpyxl")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
评估角雷达ADAS报警的KPI指标（按“博世角雷达真值系统功能验证评价标准”中“评价方法及标准”实现）。
================================================================
新增功能（2025-10-10）：
1) 目录批量：增加 --all-in-dir 选项。给定任意一个 .bag 路径后，
   会扫描“同目录下”的所有 *.bag 逐一分析，最终：
   - 输出一个“汇总CSV”（各功能行 + OVERALL，计数为所有bag的总和）
   - 额外输出一个 Excel（.xlsx），包含两个Sheet：
       · summary：与CSV完全一致（便于在Excel浏览）
       · by_bag：每个rosbag一行，列为 TP FP FN LA EA LD ED（反映每个bag的触发情况）

2) RCW 真值左右雷达合并：
   - RCW（功能索引=7）在“真值GT端”对“左右雷达”进行合并处理：
     只要左右任一雷达报，则视为GT的RCW=1（时间上做并集）。
   - WF端保持现状（不做合并；仍按当前实现读取被测topic）。

数据来源：ROS1 bag，两个topic均为 Int32MultiArray：
- 真值（RTK）：/corner_radar/sil/warning_status  （字段：data[0]=radar_id, data[1..15]=功能位）
- 被测（AUTOSAR/WF）：/corner_radar/warning_status

约定的数据布局：
- data[0] = radar_id（整型）
- data[1..15] = 15个报警功能位（0/1），顺序：
  1: LeftBsd, 2: RightBsd, 3: LeftLca, 4: RightLca, 5: LeftDow, 6: RightDow,
  7: Rcw, 8: LeftRcta, 9: RightRcta, 10: LeftRctb, 11: RightRctb,
  12: LeftFcta, 13: RightFcta, 14: LeftFctb, 15: RightFctb

评价核心（完全按PPT规则的“帧号±15帧”和“区间交叠”实现）：
----------------------------------------------------------------
1) 先把每个功能位的二值序列转为“报警区间”（连续为1的帧段）
   - 对GT得到若干 GTWARY（每个区间有：起始帧号/结束帧号/起止时间戳）
   - 对WF得到若干 WFWARY（同上）
   - “帧号”定义：以各topic自身消息顺序作为帧序号（0,1,2,...）。
     备注：GT与WF各自独立计数；PPT做的是对各自“保存结果”的帧号比较，
           在无统一帧号来源的情况下，采用各自消息序号作为“帧号”的近似。

2) 区间交叠判定（时间域交叠）：
   - 两个区间 [s1,e1] 与 [s2,e2] 只要有时间交集（max(s1,s2) < min(e1,e2)）即认为“交叠”。

3) 计数规则（严格按PPT文字实现，帧号比较均用“帧序号差”且阈值默认±15帧）：
   设“帧差容限”为 FRAME_TOL（默认15）

   A) 针对每个 WFWARY：
      - 若该 WFWARY 与某功能的多个（N≥2）GTWARY交叠：
          (1) 比较 WFWARY_STARTFRM 与“第一个交叠的GTWARY”的 GTWARY_STARTFRM：
              · 若 GTWARY_STARTFRM < WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 迟报 LA += 1
              · 若 GTWARY_STARTFRM > WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 早报 EA += 1
          (2) 比较 WFWARY_ENDFRM 与“最后一个交叠的GTWARY”的 GTWARY_ENDFRM：
              · 若 GTWARY_ENDFRM < WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 迟退 LD += 1
              · 若 GTWARY_ENDFRM > WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 早退 ED += 1
          (3) 认为“存在正报和误报场景”，即 TP += N，FP += N-1

      - 若该 WFWARY 与 0 个 GTWARY 交叠：
          → 误报 FP += 1

   B) 针对每个 GTWARY：
      - 若该 GTWARY 仅与 1 个 WFWARY 交叠：
          (1) 比较各自的起始帧：
              · 若 GTWARY_STARTFRM < WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 迟报 LA += 1
              · 若 GTWARY_STARTFRM > WFWARY_STARTFRM 且 帧差 > FRAME_TOL → 早报 EA += 1
          (2) 比较各自的结束帧：
              · 若 GTWARY_ENDFRM < WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 迟退 LD += 1
              · 若 GTWARY_ENDFRM > WFWARY_ENDFRM 且 帧差 > FRAME_TOL → 早退 ED += 1
          (3) 正报 TP += 1

      - 若该 GTWARY 与多个（N≥2） WFWARY 交叠：
          (1) 与A)相同起止帧比较（首个/最后一个）→ 可能产生LA/EA/LD/ED
          (2) 认为存在“报警间断”，DW += 1

      - 若该 GTWARY 与 0 个 WFWARY 交叠：
          → 漏报 FN += 1

5) KPI 计算：
   - FPR = FP / (FP + FN + TP)
   - FNR = FN / (FN + TP)
   - TPR = TP / (FN + TP)
   - DWR = DW / TP（当TP=0时为NaN）
   - LAR = LA / (FN + TP)
   - EAR = EA / (FN + TP)
   - LDR = LD / (FN + TP)
   - EDR = ED / (FN + TP)

输出：
- 每个功能各一行，以及 OVERALL 汇总一行，列：
  function, TP, FP, FN, LA, EA, LD, ED, FPR, FNR, TPR, DWR, LAR, EAR, LDR, EDR, gt_events, sys_events
- 当 --all-in-dir 开启时：
  · {基准bag名}_ALL_metrics.csv（所有bag合并）
  · {基准bag名}_ALL_metrics.xlsx（summary + by_bag 两个sheet）

"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

import rosbag
from std_msgs.msg import Int32MultiArray

try:
    import openpyxl
    from openpyxl import Workbook
except Exception:
    openpyxl = None  # 若环境缺少openpyxl，仍可输出CSV

# 15个功能名称（仅用于CSV/Excel可读性）
FUNC_NAMES = [
    None,
    "LeftBsd",
    "RightBsd",
    "LeftLca",
    "RightLca",
    "LeftDow",
    "RightDow",
    "Rcw",
    "LeftRcta",
    "RightRcta",
    "LeftRctb",
    "RightRctb",
    "LeftFcta",
    "RightFcta",
    "LeftFctb",
    "RightFctb",
]

RCW_INDEX = 7  # RCW 功能索引

RIGHT_FUNC_IDXS = {2, 4, 6, 9, 11, 13, 15, 7}   # 含 RCW(7)
LEFT_FUNC_IDXS  = {1, 3, 5, 8, 10, 12, 14, 7}   # 含 RCW(7)

def indices_for_side(radar_id: Optional[int]) -> set:
    # 你们的约定：3=Left, 4=Right（若未指定，就不筛）
    if radar_id == 3:
        return LEFT_FUNC_IDXS
    if radar_id == 4:
        return RIGHT_FUNC_IDXS
    return set(range(1, 16))


@dataclass
class Event:
    """单个报警区间（连续为1的帧段）"""
    start_t: float      # 起始时间戳（秒）
    end_t: float        # 结束时间戳（秒）
    start_idx: int      # 起始帧号（本topic内的消息序号）
    end_idx: int        # 结束帧号（本topic内的消息序号）

    def duration(self) -> float:
        """区间持续时长（秒）"""
        return max(0.0, self.end_t - self.start_t)


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
    将相邻事件中间仅被 <= max_gap_frames 帧的“0”间隙打断的段合并（GT端防抖）。
    gap0 = next.start_idx - prev.end_idx - 1
    当 gap0 <= max_gap_frames 时合并为一段。
    """
    if not events or max_gap_frames <= 0:
        return events
    evs = sorted(events, key=lambda e: (e.start_idx, e.start_t))
    merged: List[Event] = [Event(evs[0].start_t, evs[0].end_t, evs[0].start_idx, evs[0].end_idx)]
    for e in evs[1:]:
        last = merged[-1]
        gap0 = e.start_idx - last.end_idx - 1
        if gap0 <= max_gap_frames:
            last.end_t   = max(last.end_t, e.end_t)
            last.end_idx = max(last.end_idx, e.end_idx)
        else:
            merged.append(Event(e.start_t, e.end_t, e.start_idx, e.end_idx))
    return merged


def events_to_min_samples(evs: List[Event]) -> List[Tuple[float,int,int]]:
    """
    把事件还原成最小采样点（仅首尾两点，bit恒为1），用于复用evaluate逻辑。
    """
    samples: List[Tuple[float,int,int]] = []
    for e in evs:
        samples.append((e.start_t, 1, e.start_idx))
        samples.append((e.end_t,   1, e.end_idx))
    return samples


def safe_div(n: int, d: int) -> float:
    """安全除法，分母为0时返回NaN"""
    return float(n) / float(d) if d > 0 else float("nan")

def filter_by_radar_id(raw_all, target_id):
    """
    Return a new per-feature list using only samples of the given radar_id.
    Assumes raw_all[i] is a list of samples where each sample carries radar_id,
    e.g. dict or tuple containing 'radar_id' or similar.
    Adjust the field access to your actual structure.
    """
    out = [[] for _ in range(16)]
    for i in range(16):
        for s in raw_all[i]:
            # adapt this condition to your sample structure
            rid = s.get("radar_id", None) if isinstance(s, dict) else getattr(s, "radar_id", None)
            if rid == target_id:
                out[i].append(s)
    return out


def read_samples_with_index_GT(bag: rosbag.Bag, topic: str, radar_id: Optional[int],
                               merge_rcw_lr: bool,
                               merge_gap_frames: int = 2) -> Dict[int, List[Tuple[float, int, int]]]:
    """
    读取GT topic，提取每个功能位的三元组序列：(timestamp, bit, frame_idx)
    - 与通用读取不同点：
      · 可选：RCW（索引=7）对“左右雷达”做合并（忽略radar_id，仅要任一雷达为1即为1）
      · 其余功能位仍按传入的 radar_id 过滤（radar_id=None 则不过滤）
    - 为提升稳健性：对GT端事件可选做“短间隙合并防抖”（默认2帧）
      实现方式：先按bit序列生成事件 → 合并 → 再还原为最小采样点
    返回：dict，键=功能索引1..15，值=上述三元组列表（按时间排序）
    """
    # Step1: 读取“不过滤”的原始采样（用于RCW合并）
    raw_all: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx = -1
    for _, msg, t in bag.read_messages(topics=[topic]):
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue
        frame_idx += 1
        ts = t.to_sec()
        for i in range(1, 16):
            bit = 1 if int(data[i]) != 0 else 0
            raw_all[i].append((ts, bit, frame_idx))

    # Step2: 建立“按radar_id过滤”的采样（非RCW）
    raw_filtered: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx2 = -1
    if radar_id is None:
        # 重新遍历逐帧做 BSD/LCA 融合（而不是直接拷贝 raw_all）
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

            # 融合：BSD/LCA 互为 OR，并回写到两个功能位
            left_or  = 1 if (b1 or b3) else 0
            right_or = 1 if (b2 or b4) else 0

            # 写入 1..15（其中 1/3、2/4 用融合值，其余保持原值）
            raw_filtered[1].append((ts, left_or,  frame_idx2))   # LeftBsd
            raw_filtered[3].append((ts, left_or,  frame_idx2))   # LeftLca
            raw_filtered[2].append((ts, right_or, frame_idx2))   # RightBsd
            raw_filtered[4].append((ts, right_or, frame_idx2))   # RightLca
            for i in (5,6,7,8,9,10,11,12,13,14,15):
                bit = 1 if int(data[i]) != 0 else 0
                raw_filtered[i].append((ts, bit, frame_idx2))
    else:
        # === 修改点2：Step2 - 按指定 radar_id 过滤的情形 ===
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

    raw_filtered_id4: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx4 = -1
    for _, msg, t in bag.read_messages(topics=[topic]):
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue
        if int(data[0]) != 4:
            continue  # keep only right radar (ID=4)
        frame_idx4 += 1
        ts = t.to_sec()
        for i in range(1, 16):
            bit = 1 if int(data[i]) != 0 else 0
            raw_filtered_id4[i].append((ts, bit, frame_idx4))
    # Step3: 生成最终GT采样：
    #   · RCW：若 merge_rcw_lr=True → 使用 raw_all[RCW]（不按radar过滤）
    #   · 其余：使用 raw_filtered[i]
    out: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    for i in range(1, 16):
        if i == RCW_INDEX:
            base = sorted(raw_filtered_id4[i], key=lambda x: x[0])
        else:
            base = sorted(raw_filtered[i], key=lambda x: x[0])

        # GT端“短间隙合并防抖”：先转事件 → 合并 → 还原最小采样
        #evs = samples_to_events(base)
        #evs_m = merge_events_by_gap_frames(evs, merge_gap_frames) if merge_gap_frames > 0 else evs
        out[i] = base

    return out


def read_samples_with_index_WF(bag: rosbag.Bag, topic: str, radar_id: Optional[int]) -> Dict[int, List[Tuple[float, int, int]]]:
    """
    读取WF topic，提取每个功能位的三元组序列：(timestamp, bit, frame_idx)
    - 完全保持当前状态（不做RCW合并）
    - 若指定radar_id，则仅保留匹配的消息；否则不过滤
    """
    out: Dict[int, List[Tuple[float, int, int]]] = {i: [] for i in range(1, 16)}
    frame_idx = -1
    for _, msg, t in bag.read_messages(topics=[topic]):
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue
        if radar_id is not None and int(data[0]) != radar_id:
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

    prev_t, prev_b, prev_idx = samples[0]
    high_start_t = None
    high_start_idx = None

    # 初始化：若首帧即为1，则视为在此处上升沿
    if prev_b == 1:
        high_start_t = prev_t
        high_start_idx = prev_idx

    for t, b, idx in samples[1:]:
        if b == prev_b:
            prev_t, prev_b, prev_idx = t, b, idx
            continue
        # 发生跳变
        if prev_b == 0 and b == 1:
            # 上升沿
            high_start_t = t
            high_start_idx = idx
        elif prev_b == 1 and b == 0:
            # 下降沿，收集区间
            if high_start_t is not None and high_start_idx is not None:
                events.append(Event(start_t=high_start_t, end_t=t, start_idx=high_start_idx, end_idx=idx))
                high_start_t = None
                high_start_idx = None
        prev_t, prev_b, prev_idx = t, b, idx

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
    按PPT规则评估某一个功能位的指标计数。
    - gt_samps / wf_samps：对应功能位的 (t, bit, idx) 序列
    - frame_tol：帧号容差（±frame_tol），默认15
    返回：Counts, gt_event_count, wf_event_count
    """
    c = Counts()
    gt_events = samples_to_events(gt_samps)
    wf_events = samples_to_events(wf_samps)
    tol_sec = frame_tol * 0.066

    # 预计算交叠关系
    gt_overlaps: Dict[int, List[int]] = {i: [] for i in range(len(gt_events))}  # 每个GT区间交叠的WF区间下标
    wf_overlaps: Dict[int, List[int]] = {j: [] for j in range(len(wf_events))}  # 每个WF区间交叠的GT区间下标

    for i, g in enumerate(gt_events):
        for j, w in enumerate(wf_events):
            if intervals_overlap(g, w):
                gt_overlaps[i].append(j)
                wf_overlaps[j].append(i)

    # A) 针对每个 WFWARY 的规则
    for j, w in enumerate(wf_events):
        ovl_gt = wf_overlaps[j]
        if len(ovl_gt) == 0:
            c.FP += 1
            continue
        if len(ovl_gt) >= 2:
            first_g = gt_events[min(ovl_gt)]
            last_g  = gt_events[max(ovl_gt)]

            # 起始帧比较（迟报/早报）
            start_diff_sec = w.start_t - first_g.start_t
            if first_g.start_t < w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1
            if first_g.start_t > w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1

            # 结束帧比较（迟退/早退）
            end_diff_sec = w.end_t - last_g.end_t
            if last_g.end_t < w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1
            if last_g.end_t > w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1

            # 存在正报和误报场景（按你们PPT的“WF与多个GT交叠”解释）
            c.TP += len(ovl_gt)
            c.FP += (len(ovl_gt) - 1)

    # B) 针对每个 GTWARY 的规则
    for i, g in enumerate(gt_events):
        ovl_wf = gt_overlaps[i]
        if len(ovl_wf) == 0:
            c.FN += 1
            continue
        if len(ovl_wf) == 1:
            w = wf_events[ovl_wf[0]]

            start_diff_sec = w.start_t - g.start_t
            if g.start_t < w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1
            if g.start_t > w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1

            end_diff_sec = w.end_t - g.end_t
            if g.end_t < w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1
            if g.end_t > w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1

            
            c.TP += 1

        if len(ovl_wf) >= 2:
            first_w = wf_events[min(ovl_wf)]
            last_w  = wf_events[max(ovl_wf)]

            start_diff_sec = first_w.start_t - g.start_t
            if g.start_t < first_w.start_t and abs(start_diff_sec) > tol_sec:
                c.LA += 1
            if g.start_t > first_w.start_t and abs(start_diff_sec) > tol_sec:
                c.EA += 1

            end_diff_sec = last_w.end_t - g.end_t
            if g.end_t < last_w.end_t and abs(end_diff_sec) > tol_sec:
                c.LD += 1
            if g.end_t > last_w.end_t and abs(end_diff_sec) > tol_sec:
                c.ED += 1

            c.TP += 1
            c.DW += 1

    return c, len(gt_events), len(wf_events)


def write_csv(out_path: str, rows: List[Dict[str, object]]) -> None:
    """写CSV工具"""
    fieldnames = list(rows[0].keys()) if rows else []
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


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
    评估单个bag，返回：
    - rows：每个功能一行 + OVERALL（同CSV结构）
    - total_counts：该bag自身的 OVERALL 计数（仅计TP/FP/FN/LA/EA/LD/ED/DW）
    """
    bag = rosbag.Bag(bag_path, "r")

    # GT：带RCW左右合并（仅RCW），并做短间隙合并防抖
    gt_samples_all = read_samples_with_index_GT(bag, gt_topic, radar_id_gt,
                                                merge_rcw_lr=merge_rcw_lr,
                                                merge_gap_frames=merge_gap_frames)

    # WF：保持当前实现（不合并RCW）
    wf_samples_all = read_samples_with_index_WF(bag, wf_topic, 4)

    rows: List[Dict[str, object]] = []
    total = Counts()
    total_gt_events = 0
    total_wf_events = 0

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
            "function": name,
            "TP": c.TP,
            "FP": c.FP,
            "FN": c.FN,
            "LA": c.LA,
            "EA": c.EA,
            "LD": c.LD,
            "ED": c.ED,
            "DW": c.DW,
            "FPR": f"{safe_div(c.FP, denom_all):.6f}",
            "FNR": f"{safe_div(c.FN, denom_pos):.6f}",
            "TPR": f"{safe_div(c.TP, denom_pos):.6f}",
            "DWR": f"{safe_div(c.DW, c.TP):.6f}" if c.TP > 0 else "nan",
            "LAR": f"{safe_div(c.LA, denom_pos):.6f}",
            "EAR": f"{safe_div(c.EA, denom_pos):.6f}",
            "LDR": f"{safe_div(c.LD, denom_pos):.6f}",
            "EDR": f"{safe_div(c.ED, denom_pos):.6f}",
            "gt_events": gt_ev_cnt,
            "sys_events": wf_ev_cnt,
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
    parser = argparse.ArgumentParser(description="角雷达报警KPI评估（支持目录批量与RCW左右合并）。")
    #parser.add_argument("bag", help="任意一个rosbag文件路径（若使用 --all-in-dir，则会扫描其所在目录）")
    parser.add_argument("--gt", default="/corner_radar/sil/warning_status", help="真值topic")
    parser.add_argument("--sys", default="/corner_radar/warning_status", help="被测topic")
    parser.add_argument("--radar-id", type=int, default=None, help="GT侧：除RCW外的功能，若指定则仅评估该radar_id（RCW始终做左右合并）")
    parser.add_argument("--radar-id-wf", type=int, default=4, help="WF侧：被测topic过滤的radar_id（默认4，按你们现状）")
    parser.add_argument("--frame-tol", type=int, default=15, help="帧号容差（±frame-tol），默认15")
    parser.add_argument("--merge-gap-frames", type=int, default=2, help="GT端事件去抖合并：相邻两段之间的“0”间隙帧数 ≤ 该值时合并（默认2；设为0则不合并）")
    #parser.add_argument("--all-in-dir", action="store_true", help="扫描bag所在目录下的所有*.bag并合并统计")
    parser.add_argument("--out", default=None, help="输出CSV路径（默认：单bag为 *_metrics.csv；目录合并为 *_ALL_metrics.csv）")
    parser.add_argument("--xlsx", default=None, help="目录合并时的Excel输出路径（默认：*_ALL_metrics.xlsx）")
    args = parser.parse_args()

    
    base_dir = os.getcwd()
    #base_name_noext = os.path.splitext(os.path.basename(bag_path))[0]

    

    # 目录批量：搜集所有*.bag
    bag_files = [os.path.join(base_dir, f) for f in os.listdir(base_dir) if f.lower().endswith(".bag")]
    bag_files.sort()

    if not bag_files:
        print(f"[WARN] 目录 {base_dir} 未发现任何 .bag 文件。")
        return

    print(f"[INFO] 扫描到 {len(bag_files)} 个bag：")
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

    # 写CSV（汇总）
    #out_csv = args.out or os.path.join(base_dir, f"{base_name_noext}_ALL_metrics.csv")
    #write_csv(out_csv, summary_rows)
    #print(f"已输出汇总CSV：{out_csv}")

    # 写Excel（两个sheet）
    if args.radar_id == 3:
        excel_name = "Left radar.xlsx"
    elif args.radar_id == 4:
        excel_name = "Right radar.xlsx"
    else:
        excel_name = f"radar_{args.radar_id}.xlsx"
    out_xlsx = os.path.join(base_dir, excel_name)
    write_xlsx(out_xlsx, summary_rows, by_bag_rows)
    if openpyxl is not None:
        print(f"已输出Excel：{out_xlsx} （summary + by_bag）")
    else:
        print("[WARN] 当前环境未安装 openpyxl，已跳过Excel导出。可：pip install openpyxl")

if __name__ == "__main__":
    main()

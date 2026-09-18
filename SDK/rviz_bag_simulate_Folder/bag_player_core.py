#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
后端：bag 读取、消息发布、播放控制、/play_single_frame_ 服务。不含任何 UI 代码。

对外接口：
  - 回调：on_progress / on_frame / on_frame_info / on_bag_loaded / on_play_finished(serial)（可能在工作线程触发）
  - 方法：load_bag / jump / play / stop / shutdown
  - 属性：playing / stop_requested / play_serial / sp_flag / main_radar_index / play_rate / current_frame

文件夹顺序播放由 UI 层编排：每次只加载/播放一个 bag，播完后回调再加载下一个。
"""

import os
import time
import threading
from datetime import datetime, timezone, timedelta

import rospy
import rosbag

from sensor_msgs.msg import CompressedImage
from arbe_msgs.msg import wfRawDataMsg, wfTiFrameRD, wfSimulateData, VehStatusOutput
from wf_srvs.srv import PlaySingleFrame, PlaySingleFrameResponse

PCL_TOPICS = ['/wf/corner_radar/parsed/float_data_%d' % radar_idx for radar_idx in range(5)]
SP_TOPICS = ['/wf/frame_rd_data/ti/radar_%d' % radar_idx for radar_idx in range(5)]
SIM_TOPICS = ['/wf/corner_radar/simulate_data_%d' % radar_idx for radar_idx in range(5)]
CAM_TOPICS = ['/cv_camera_%d/image_raw/compressed' % cam_idx for cam_idx in range(6)]
CAR_TOPIC = '/wf/car_id6/parsed2'

ALL_TOPICS = PCL_TOPICS + SP_TOPICS + SIM_TOPICS + CAM_TOPICS + [CAR_TOPIC]
PCL_SET = set(PCL_TOPICS)
SP_SET = set(SP_TOPICS)
SIM_SET = set(SIM_TOPICS)
CAM_SET = set(CAM_TOPICS)


def _instantiate(entry):
    """entry = (t, pytype, data)。发布时才反序列化（等价于 C++ 的 MessageInstance::instantiate）"""
    msg = entry[1]()
    msg.deserialize(entry[2])
    return msg


def _find_closest(msgs, selected_time):
    """线性查找时间最近的消息下标（与 C++ 行为一致），空表返回 -1"""
    if not msgs:
        return -1
    best_idx = -1
    best_diff = None
    sel_sec = selected_time.to_sec()
    for idx, entry in enumerate(msgs):
        diff = abs(sel_sec - entry[0].to_sec())
        if best_diff is None or diff < best_diff:
            best_diff = diff
            best_idx = idx
    return best_idx


class BagLoader(object):
    """读取 bag 中指定 topic 的原始数据，按 topic 分类缓存"""

    def __init__(self):
        self.clear()

    def clear(self):
        self.pcl = [[] for _ in range(5)]   # wfRawDataMsg
        self.sp = [[] for _ in range(5)]    # wfTiFrameRD
        self.sim = [[] for _ in range(5)]   # wfSimulateData
        self.cam = [[] for _ in range(6)]   # CompressedImage
        self.car = []                       # VehStatusOutput

    def load(self, path, progress_cb=None, abort_cb=None):
        """progress_cb: 0.0~1.0；abort_cb 返回 True 时中止读取"""
        self.clear()
        if progress_cb:
            progress_cb(0.05)
        with rosbag.Bag(path, 'r') as bag:
            if progress_cb:
                progress_cb(0.1)
            total = max(bag.get_message_count(topic_filters=ALL_TOPICS), 1)
            if progress_cb:
                progress_cb(0.2)
            count = 0
            for topic, raw, t in bag.read_messages(topics=ALL_TOPICS, raw=True):
                if abort_cb and abort_cb():
                    raise RuntimeError('load aborted')
                pytype = raw[-1]
                entry = (t, pytype, raw[1])
                if topic == CAR_TOPIC:
                    self.car.append(entry)
                elif topic in PCL_SET:
                    self.pcl[PCL_TOPICS.index(topic)].append(entry)
                elif topic in SP_SET:
                    self.sp[SP_TOPICS.index(topic)].append(entry)
                elif topic in SIM_SET:
                    self.sim[SIM_TOPICS.index(topic)].append(entry)
                elif topic in CAM_SET:
                    self.cam[CAM_TOPICS.index(topic)].append(entry)
                count += 1
                if progress_cb and (count % 500 == 0 or count >= total):
                    progress_cb(0.2 + 0.8 * count / total)
        if progress_cb:
            progress_cb(1.0)


class BagPlayer(object):
    """播放控制核心：发布话题、注册服务、播放线程"""

    def __init__(self):
        self.loader = BagLoader()
        self.playing = False
        self.stop_requested = False
        self.finish_event = threading.Event()
        self.finish_event.set()   # 对应 C++ finishProcessFlag_ 初始为 true
        self.sp_flag = False      # 对应 C++ bSPFlag / bPlaySPFlag_
        self.main_radar_index = 1
        self.play_rate = 1.0
        self.current_frame = 0
        self._simulate_frame_id = 0
        self._play_thread = None
        self.play_serial = 0        # 播放代号：play() 自增，用于丢弃迟到的 on_play_finished

        # UI 注册的回调（在工作线程中被调用，UI 侧自行做线程封送）
        self.on_progress = None       # (float 0~1) 读取进度
        self.on_frame = None          # (int) 当前播放帧号
        self.on_frame_info = None      # (str) 帧信息文本
        self.on_bag_loaded = None     # (dict) 加载开始/完成
        self.on_play_finished = None  # (serial) 当前 bag 播放结束（自然播完或被 stop）

        # 发布器
        self.pcl_pubs = [rospy.Publisher(t, wfRawDataMsg, queue_size=10) for t in PCL_TOPICS]
        self.sp_pubs = [rospy.Publisher(t, wfTiFrameRD, queue_size=10) for t in SP_TOPICS]
        self.sim_pubs = [rospy.Publisher(t, wfSimulateData, queue_size=10) for t in SIM_TOPICS]
        self.cam_pubs = [rospy.Publisher(t, CompressedImage, queue_size=10) for t in CAM_TOPICS]
        self.car_pub = rospy.Publisher(CAR_TOPIC, VehStatusOutput, queue_size=1)

        # 服务（与 C++ 版一致：5 个服务同一处理函数）
        for radar_idx in range(5):
            rospy.Service('/play_single_frame_%d' % radar_idx, PlaySingleFrame, self._handle_service)

    # ---------- 查询 ----------
    def pcl_counts(self):
        return [len(x) for x in self.loader.pcl]

    def main_count(self):
        return len(self._main_msgs())

    def _main_msgs(self):
        if self.main_radar_index < 0 or self.main_radar_index > 4:
            return []
        return self.loader.sp[self.main_radar_index] if self.sp_flag else self.loader.pcl[self.main_radar_index]

    def _emit(self, cb, *args):
        if cb is not None:
            cb(*args)

    # ---------- 服务 ----------
    def _handle_service(self, req):
        # 与 C++ 版一致：radar_pos 与当前主雷达一致时才处理；status=1 表示下游处理完当前帧
        if self.main_radar_index == req.radar_pos:
            if req.status == 1:
                self.finish_event.set()
            elif req.status != 0:
                rospy.loginfo('Play single frame service error!')
            return PlaySingleFrameResponse(success=True)
        return PlaySingleFrameResponse(success=False)

    # ---------- 发布 ----------
    def _publish_closest(self, selected_time):
        """与 C++ publishClosestMessages 顺序一致：车况 -> 模拟数据 -> 8ms -> 点云/SP点云 -> 相机"""
        loader = self.loader

        car_idx = _find_closest(loader.car, selected_time)
        if car_idx >= 0:
            self.car_pub.publish(_instantiate(loader.car[car_idx]))

        if not self.sp_flag:
            for radar_idx in range(5):
                msg_idx = _find_closest(loader.sim[radar_idx], selected_time)
                if msg_idx < 0:
                    continue
                msg = _instantiate(loader.sim[radar_idx][msg_idx])
                if radar_idx == 1:
                    self._simulate_frame_id = msg.frameID
                    rospy.loginfo(' \n ')
                    rospy.loginfo('simulate_data1 frame_id:: %d', self._simulate_frame_id)
                self.sim_pubs[radar_idx].publish(msg)

            time.sleep(0.001 * 6)

            for radar_idx in range(5):
                msg_idx = _find_closest(loader.pcl[radar_idx], selected_time)
                if msg_idx < 0:
                    continue
                msg = _instantiate(loader.pcl[radar_idx][msg_idx])
                if radar_idx == 1:
                    custom_frame_id1 = msg.frame_id
                    rospy.loginfo('Pointcloud1 custom frame_id: %d', custom_frame_id1)
                    if self._simulate_frame_id != custom_frame_id1:
                        rospy.logerr('Frame ID mismatch: simulate_frame_id=%d, custom_frame_id1=%d',
                                     self._simulate_frame_id, custom_frame_id1)
                if radar_idx == 3:
                    self._report_frame_info(msg.frame_id, msg.header.stamp)
                    rospy.loginfo('Pointcloud custom frame_id: %d', msg.frame_id)
                self.pcl_pubs[radar_idx].publish(msg)
        else:
            for radar_idx in range(5):
                msg_idx = _find_closest(loader.sp[radar_idx], selected_time)
                if msg_idx < 0:
                    continue
                msg = _instantiate(loader.sp[radar_idx][msg_idx])
                if radar_idx == 3:
                    self._report_frame_info(msg.frameID, msg.header.stamp)
                    rospy.loginfo('Pointcloud3 custom frame_id: %d', msg.frameID)
                self.sp_pubs[radar_idx].publish(msg)

        for cam_idx in range(6):
            msg_idx = _find_closest(loader.cam[cam_idx], selected_time)
            if msg_idx >= 0:
                self.cam_pubs[cam_idx].publish(_instantiate(loader.cam[cam_idx][msg_idx]))

    def _report_frame_info(self, frame_id, stamp):
        # 时间戳 +8 小时（与 C++ time_offset 一致）
        try:
            dt = datetime.fromtimestamp(stamp.to_sec(), tz=timezone.utc) + timedelta(hours=8)
            time_str = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
        except Exception:
            time_str = str(stamp)
        self._emit(self.on_frame_info, '-Frame ID: %d  -Timestamp: %s' % (frame_id, time_str))

    # ---------- 单文件加载 / 跳帧 ----------
    def load_bag(self, path):
        """在调用线程（通常是 UI 线程）同步加载一个 bag"""
        self.loader.load(path, progress_cb=lambda v: self._emit(self.on_progress, v))
        self.current_frame = 0
        self._emit(self.on_bag_loaded, {
            'pcl': self.pcl_counts(),
            'file': os.path.basename(path),
            'path': path,
            'enable_ui': True,
        })

    def jump(self, frame_number):
        main = self._main_msgs()
        if 0 <= frame_number < len(main):
            self.current_frame = frame_number
            self._publish_closest(main[frame_number][0])
            self._emit(self.on_frame, frame_number)
            rospy.loginfo('Playing frame %d', frame_number)

    # ---------- 播放 ----------
    def play(self):
        """播放当前已加载的 bag（从 current_frame 之后继续），只负责这一个 bag；
        播完（或被 stop）后回调 on_play_finished(serial)，由上层决定是否加载下一个"""
        self.play_serial += 1
        self.playing = True
        self.stop_requested = False
        self.finish_event.set()   # 不携带上次播放遗留的 clear，避免第一帧死等应答
        self._play_thread = threading.Thread(target=self._play_worker, daemon=True)
        self._play_thread.start()

    def _play_worker(self):
        serial = self.play_serial
        try:
            self._play_loop()   # 自然播完时把 playing 置 False
        finally:
            self._emit(self.on_play_finished, serial)

    def _play_loop(self, from_frame=None):
        main = self._main_msgs()
        total_frames = len(main)
        frame_idx = self.current_frame if from_frame is None else from_frame
        if from_frame is None and frame_idx != 0:
            frame_idx += 1   #从当前位置继续时跳过当前帧
        while frame_idx < total_frames:
            while not self.finish_event.is_set():
                if not self.playing:
                    break
                time.sleep(0.001)
            if not self.playing:
                break
            # 先消费应答再发布：clear 若放在发布之后，发布期间（含内部 8ms sleep）到达的快速应答会被 clear 抹掉，导致下一帧死等
            self.finish_event.clear()
            self.current_frame = frame_idx
            self._publish_closest(main[frame_idx][0])
            self._emit(self.on_frame, frame_idx)
            time.sleep(0.005 / self.play_rate)
            frame_idx += 1
        self.playing = False

    def stop(self):
        self.playing = False
        self.stop_requested = True
        self.finish_event.set()

    def shutdown(self):
        """停止播放并等待线程退出（阻塞）；不清除已加载的数据"""
        self.stop()
        if self._play_thread is not None and self._play_thread.is_alive():
            self._play_thread.join(5.0)
        self._play_thread = None
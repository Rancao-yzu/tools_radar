#!/usr/bin/env python
# BagReader: 读取并缓存 ROS bag 文件，提供逐帧播放能力
# 核心机制：以主雷达时间戳为基准，查找各话题时间最接近的消息作为一帧
# 播放同步：通过 threading.Event 实现外部服务逐帧控制（PlaySingleFrame）
import rosbag
import rospy
import threading
import time

class BagReader:
    """
    bag 文件读取与帧播放控制器

    --- 数据流 ---
    read_bag_file() → 全量缓存到内存（各话题独立的列表）
    play_bag()      → 启动播放线程 _play_loop()
    _play_loop()    → 每帧：wait event → 查找最接近消息 → 组装 → 回调 → clear event → sleep

    --- 同步机制 ---
    finish_process_event_ (threading.Event):
      - 初始 set()，确保首帧不被阻塞
      - 每帧发布前 clear()，发布后 wait()
      - 外部 PlaySingleFrame 服务调用 set_finish_process_flag(True) 来 set()，唤醒下一帧
      - stop_bag() 也会 set()，让等待中的线程退出

    --- msg_flags_ 索引约定 ---
    数组长度 14，每个元素是某话题在该帧中的消息索引，-1 表示该话题无数据：
    MAX_TOPIC_NUM = 14   # 0-4:点云 5:warning 6-11:相机 12:车辆 13:IMU
    """
    MAX_TOPIC_NUM = 14

    def __init__(self):
        # ---- 播放状态 ----
        self.current_frame_ = 0          # 当前帧序号（以主雷达点云索引为基准）
        self.play_rate_ = 1.0            # 播放倍速（1.0=50ms/帧, 2.0=25ms/帧）
        self.playing_ = False            # 是否正在播放
        self.main_radar_index_ = 3       # 当前选中的主雷达（默认3=后左角）
        self.play_thread_ = None         # 播放工作线程
        self.mutex_ = threading.Lock()   # 保护 playing_ 和帧数据的互斥锁

        # ---- 回调函数 ----
        self.message_callback_ = None               # 每帧组装完成后调用，传入 (frame_msgs, frame_number, msg_flags)
        self.update_progress_bar_callback_ = None   # 读取进度回调，传入 (float 0.0~1.2)

        # ---- 同步原语：对应 C++ 原版 finishProcessFlag_ ----
        self.finish_process_event_ = threading.Event()
        self.finish_process_event_.set()   # 初始置位，首帧可直接播放

        # ---- 消息缓存：按话题分类存储，每条消息是 (topic, msg, timestamp) 三元组 ----
        self.car_msgs_ = []                   # [12] 车辆状态
        self.camera_msgs0_ = []               # [6]  相机0
        self.camera_msgs1_ = []               # [7]  相机1
        self.camera_msgs2_ = []               # [8]  相机2
        self.camera_msgs3_ = []               # [9]  相机3
        self.camera_msgs4_ = []               # [10] 相机4
        self.camera_msgs5_ = []               # [11] 相机5
        self.pointcloud_msgs0_ = []           # [0]  前雷达点云
        self.pointcloud_msgs1_ = []           # [1]  前左角雷达点云
        self.pointcloud_msgs2_ = []           # [2]  前右角雷达点云
        self.pointcloud_msgs3_ = []           # [3]  后左角雷达点云
        self.pointcloud_msgs4_ = []           # [4]  后右角雷达点云
        self.corner_radar_warning_msgs_ = []  # [5]  角雷达警告
        self.IMU_msgs_ = []                   # [13] IMU

        # ---- 帧组装相关 ----
        self.empty_msgs_ = []           # 占位空消息，当某话题无数据时填充
        self.frame_msgs_ = []           # 当前帧的完整消息列表（14个元素）
        self.msg_flags_ = [-1] * 14     # 每话题在当前帧中的索引，-1 表示无数据

    # ================================================================
    #  公开接口
    # ================================================================

    def read_bag_file(self, file_path):
        """
        读取 bag 文件，按话题分类缓存所有消息到内存。

        步骤：
          1. 停止当前播放
          2. 清空所有缓存列表
          3. 打开 bag，遍历指定话题
          4. 按话题名分拣到对应列表
          5. 取第一个有效点云消息作为占位消息
          6. 当前帧复位为 0

        返回：(count0, count1, count2, count3, count4) 各雷达点云数量
        """
        if self.update_progress_bar_callback_:
            self.update_progress_bar_callback_(0.05)

        self.stop_bag()

        # 清空所有缓存列表
        self.car_msgs_ = []
        self.camera_msgs0_ = []
        self.camera_msgs1_ = []
        self.camera_msgs2_ = []
        self.camera_msgs3_ = []
        self.camera_msgs4_ = []
        self.camera_msgs5_ = []
        self.pointcloud_msgs0_ = []
        self.pointcloud_msgs1_ = []
        self.pointcloud_msgs2_ = []
        self.pointcloud_msgs3_ = []
        self.pointcloud_msgs4_ = []
        self.corner_radar_warning_msgs_ = []
        self.IMU_msgs_ = []
        self.empty_msgs_ = []
        self.msg_flags_ = [-1] * 14

        if self.update_progress_bar_callback_:
            self.update_progress_bar_callback_(0.1)

        bag = rosbag.Bag(file_path, 'r')

        if self.update_progress_bar_callback_:
            self.update_progress_bar_callback_(0.15)

        # 需要读取的话题列表（与 C++ 原版完全一致）
        topics = [
            '/wf/corner_radar/lgu_data_0',
            '/wf/corner_radar/lgu_data_1',
            '/wf/corner_radar/lgu_data_2',
            '/wf/corner_radar/lgu_data_3',
            '/wf/corner_radar/lgu_data_4',
            '/cv_camera_0/image_raw/compressed',
            '/cv_camera_1/image_raw/compressed',
            '/cv_camera_2/image_raw/compressed',
            '/cv_camera_3/image_raw/compressed',
            '/cv_camera_4/image_raw/compressed',
            '/cv_camera_5/image_raw/compressed',
            '/wf/car_id6/parsed2',
            '/corner_radar/warning_status',
            '/wf/imu_data/parsed'
        ]

        if self.update_progress_bar_callback_:
            self.update_progress_bar_callback_(0.2)

        count = 0
        total_count = bag.get_message_count(topic_filters=topics)

        # 遍历 bag 中的每条消息，按话题分拣到对应列表
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = (topic, msg, t)
            if topic == '/wf/car_id6/parsed2':
                self.car_msgs_.append(entry)
            elif topic == '/wf/corner_radar/lgu_data_0':
                self.pointcloud_msgs0_.append(entry)
            elif topic == '/wf/corner_radar/lgu_data_1':
                self.pointcloud_msgs1_.append(entry)
            elif topic == '/wf/corner_radar/lgu_data_2':
                self.pointcloud_msgs2_.append(entry)
            elif topic == '/wf/corner_radar/lgu_data_3':
                self.pointcloud_msgs3_.append(entry)
            elif topic == '/wf/corner_radar/lgu_data_4':
                self.pointcloud_msgs4_.append(entry)
            elif topic == '/corner_radar/warning_status':
                self.corner_radar_warning_msgs_.append(entry)
            elif topic == '/cv_camera_0/image_raw/compressed':
                self.camera_msgs0_.append(entry)
            elif topic == '/cv_camera_1/image_raw/compressed':
                self.camera_msgs1_.append(entry)
            elif topic == '/cv_camera_2/image_raw/compressed':
                self.camera_msgs2_.append(entry)
            elif topic == '/cv_camera_3/image_raw/compressed':
                self.camera_msgs3_.append(entry)
            elif topic == '/cv_camera_4/image_raw/compressed':
                self.camera_msgs4_.append(entry)
            elif topic == '/cv_camera_5/image_raw/compressed':
                self.camera_msgs5_.append(entry)
            elif topic == '/wf/imu_data/parsed':
                self.IMU_msgs_.append(entry)

            count += 1
            if self.update_progress_bar_callback_:
                self.update_progress_bar_callback_(count / total_count + 0.2)

        bag.close()
        self.current_frame_ = 0

        # 取第一个有效点云消息作为占位消息
        # 当某话题在当前帧中无匹配数据时，用此消息填充
        if len(self.pointcloud_msgs0_) > 0:
            self.empty_msgs_.append(self.pointcloud_msgs0_[0])
        elif len(self.pointcloud_msgs1_) > 0:
            self.empty_msgs_.append(self.pointcloud_msgs1_[0])
        elif len(self.pointcloud_msgs2_) > 0:
            self.empty_msgs_.append(self.pointcloud_msgs2_[0])
        elif len(self.pointcloud_msgs3_) > 0:
            self.empty_msgs_.append(self.pointcloud_msgs3_[0])
        elif len(self.pointcloud_msgs4_) > 0:
            self.empty_msgs_.append(self.pointcloud_msgs4_[0])

        return (len(self.pointcloud_msgs0_), len(self.pointcloud_msgs1_),
                len(self.pointcloud_msgs2_), len(self.pointcloud_msgs3_),
                len(self.pointcloud_msgs4_))

    def get_main_radar_pcl_size(self):
        """
        获取当前主雷达的点云消息数量。
        这是播放的总帧数上限——以主雷达消息数量为准遍历其他话题。
        """
        sizes = [
            len(self.pointcloud_msgs0_),
            len(self.pointcloud_msgs1_),
            len(self.pointcloud_msgs2_),
            len(self.pointcloud_msgs3_),
            len(self.pointcloud_msgs4_)
        ]
        return sizes[self.main_radar_index_] if self.main_radar_index_ < 5 else len(self.pointcloud_msgs4_)

    def get_main_radar_time(self, cur_idx):
        """
        获取主雷达第 cur_idx 条消息的时间戳。
        所有话题都以这个时间戳为基准查找各自最接近的消息。
        """
        msgs_list = [
            self.pointcloud_msgs0_,
            self.pointcloud_msgs1_,
            self.pointcloud_msgs2_,
            self.pointcloud_msgs3_,
            self.pointcloud_msgs4_
        ]
        idx = self.main_radar_index_ if self.main_radar_index_ < 5 else 4
        if 0 <= cur_idx < len(msgs_list[idx]):
            return msgs_list[idx][cur_idx][2]   # entry[2] 是 rospy.Time
        return rospy.Time(0)

    def packet_callback_msg(self):
        """
        根据 msg_flags_ 索引数组组装当前帧的完整消息列表。

        - msg_flags_[i] >= 0 → 从对应话题列表中取出该索引的消息
        - msg_flags_[i] < 0  → 该话题无匹配数据，用占位消息填充
        """
        self.frame_msgs_ = []
        for i in range(self.MAX_TOPIC_NUM):
            if self.msg_flags_[i] < 0:
                if self.empty_msgs_:
                    self.frame_msgs_.append(self.empty_msgs_[0])
                else:
                    self.frame_msgs_.append(None)
            else:
                if i == 0:
                    self.frame_msgs_.append(self.pointcloud_msgs0_[self.msg_flags_[i]])
                elif i == 1:
                    self.frame_msgs_.append(self.pointcloud_msgs1_[self.msg_flags_[i]])
                elif i == 2:
                    self.frame_msgs_.append(self.pointcloud_msgs2_[self.msg_flags_[i]])
                elif i == 3:
                    self.frame_msgs_.append(self.pointcloud_msgs3_[self.msg_flags_[i]])
                elif i == 4:
                    self.frame_msgs_.append(self.pointcloud_msgs4_[self.msg_flags_[i]])
                elif i == 5:
                    self.frame_msgs_.append(self.corner_radar_warning_msgs_[self.msg_flags_[i]])
                elif i == 6:
                    self.frame_msgs_.append(self.camera_msgs0_[self.msg_flags_[i]])
                elif i == 7:
                    self.frame_msgs_.append(self.camera_msgs1_[self.msg_flags_[i]])
                elif i == 8:
                    self.frame_msgs_.append(self.camera_msgs2_[self.msg_flags_[i]])
                elif i == 9:
                    self.frame_msgs_.append(self.camera_msgs3_[self.msg_flags_[i]])
                elif i == 10:
                    self.frame_msgs_.append(self.camera_msgs4_[self.msg_flags_[i]])
                elif i == 11:
                    self.frame_msgs_.append(self.camera_msgs5_[self.msg_flags_[i]])
                elif i == 12:
                    self.frame_msgs_.append(self.car_msgs_[self.msg_flags_[i]])
                elif i == 13:
                    self.frame_msgs_.append(self.IMU_msgs_[self.msg_flags_[i]])
                else:
                    self.frame_msgs_.append(None)

    def jump_to_frame(self, frame_number):
        """跳到指定帧：以主雷达时间戳为基准，查找各话题最接近的消息"""
        main_radar_size = self.get_main_radar_pcl_size()
        if 0 <= frame_number < main_radar_size:
            self.current_frame_ = frame_number
            if self.message_callback_:
                selected_time = self.get_main_radar_time(self.current_frame_)

                # 以主雷达时间戳为基准，为每个话题查找最接近的消息
                self.msg_flags_[0] = self._find_closest_frame(selected_time, self.pointcloud_msgs0_)
                self.msg_flags_[1] = self._find_closest_frame(selected_time, self.pointcloud_msgs1_)
                self.msg_flags_[2] = self._find_closest_frame(selected_time, self.pointcloud_msgs2_)
                self.msg_flags_[3] = self._find_closest_frame(selected_time, self.pointcloud_msgs3_)
                self.msg_flags_[4] = self._find_closest_frame(selected_time, self.pointcloud_msgs4_)
                self.msg_flags_[5] = self._find_closest_frame(selected_time, self.corner_radar_warning_msgs_)
                self.msg_flags_[6] = self._find_closest_frame(selected_time, self.camera_msgs0_)
                self.msg_flags_[7] = self._find_closest_frame(selected_time, self.camera_msgs1_)
                self.msg_flags_[8] = self._find_closest_frame(selected_time, self.camera_msgs2_)
                self.msg_flags_[9] = self._find_closest_frame(selected_time, self.camera_msgs3_)
                self.msg_flags_[10] = self._find_closest_frame(selected_time, self.camera_msgs4_)
                self.msg_flags_[11] = self._find_closest_frame(selected_time, self.camera_msgs5_)
                self.msg_flags_[12] = self._find_closest_frame(selected_time, self.car_msgs_)
                self.msg_flags_[13] = self._find_closest_frame(selected_time, self.IMU_msgs_)

                self.packet_callback_msg()
                self.message_callback_(self.frame_msgs_, self.current_frame_, self.msg_flags_)
                rospy.loginfo("Playing frame %d", self.current_frame_)

    def play_bag(self):
        """
        开始播放：将 playing_ 置为 true，启动后台播放线程。
        线程入口 _play_loop 会从 current_frame_ 开始逐帧推进。
        """
        if not self.playing_:
            self.playing_ = True
            self.play_thread_ = threading.Thread(target=self._play_loop)
            self.play_thread_.start()

    def stop_bag(self):
        """
        停止播放：将 playing_ 置为 false，set event 解除可能的 wait 阻塞，
        然后 join 等待播放线程退出。
        """
        if self.playing_:
            self.playing_ = False
            self.finish_process_event_.set()   # 唤醒可能正在 wait() 的线程
            if self.play_thread_ and self.play_thread_.is_alive():
                self.play_thread_.join()

    def set_play_rate(self, rate):
        self.play_rate_ = rate

    def set_message_callback(self, callback):
        """
        设置消息回调。
        每帧组装完成后调用 callback(frame_msgs, frame_number, msg_flags)。
        """
        self.message_callback_ = callback

    def set_update_progress_bar_callback(self, callback):
        self.update_progress_bar_callback_ = callback

    def get_current_frame(self):
        return self.current_frame_

    def select_main_radar(self, index):
        """
        切换主雷达。
        index: 0=前雷达, 1=前左角, 2=前右角, 3=后左角, 4=后右角
        """
        self.main_radar_index_ = index

    def set_finish_process_flag(self, flag):
        """
        由外部 PlaySingleFrame 服务回调。
        flag=True → set event → 播放线程从 wait() 中唤醒，继续下一帧。

        与 C++ 原版 setFinishProcessFlag(bool) 完全对应。
        """
        if flag:
            self.finish_process_event_.set()

    # ================================================================
    #  内部方法
    # ================================================================

    def _find_closest_frame(self, selected_time, msg_list):
        """
        在 msg_list 中查找时间戳最接近 selected_time 的消息索引。

        参数:
          selected_time: rospy.Time，基准时间戳（主雷达当前消息的时间）
          msg_list:       [(topic, msg, time), ...] 某话题的全部消息

        返回:
          最接近的消息索引，列表为空时返回 -1
        """
        if not msg_list:
            return -1
        closest_index = -1
        min_diff = float('inf')
        for i, entry in enumerate(msg_list):
            msg_time = entry[2]   # entry 是 (topic, msg, time) 三元组
            time_diff = abs((selected_time - msg_time).to_sec())
            if time_diff < min_diff:
                min_diff = time_diff
                closest_index = i
        return closest_index

    def _play_loop(self):
        """
        播放线程主循环。

        流程：
          1. 等待 finish_process_event_（外部服务 set 才放行）
          2. 检查 playing_ 标志
          3. 以主雷达时间戳为基准查找各话题最接近消息
          4. 组装帧 → 回调发布（clear event 阻塞等待服务响应）
          5. sleep(50ms / play_rate)

        外部服务调用 set_finish_process_flag(True) → set() → 解除阻塞 → 下一帧。
        """
        main_radar_size = self.get_main_radar_pcl_size()

        # 如果当前帧不为 0（例如先 jump_to_frame 再 play），从下一帧开始
        if self.current_frame_ != 0:
            self.current_frame_ += 1

        for i in range(self.current_frame_, main_radar_size):

            # ===== 阻塞等待外部 PlaySingleFrame 服务的完成信号 =====
            self.finish_process_event_.wait()

            with self.mutex_:
                if not self.playing_:
                    break

                self.current_frame_ = i
                selected_time = self.get_main_radar_time(self.current_frame_)

                # 以主雷达时间戳为基准，为每个话题查找最接近的消息
                self.msg_flags_[0] = self._find_closest_frame(selected_time, self.pointcloud_msgs0_)
                self.msg_flags_[1] = self._find_closest_frame(selected_time, self.pointcloud_msgs1_)
                self.msg_flags_[2] = self._find_closest_frame(selected_time, self.pointcloud_msgs2_)
                self.msg_flags_[3] = self._find_closest_frame(selected_time, self.pointcloud_msgs3_)
                self.msg_flags_[4] = self._find_closest_frame(selected_time, self.pointcloud_msgs4_)
                self.msg_flags_[5] = self._find_closest_frame(selected_time, self.corner_radar_warning_msgs_)
                self.msg_flags_[6] = self._find_closest_frame(selected_time, self.camera_msgs0_)
                self.msg_flags_[7] = self._find_closest_frame(selected_time, self.camera_msgs1_)
                self.msg_flags_[8] = self._find_closest_frame(selected_time, self.camera_msgs2_)
                self.msg_flags_[9] = self._find_closest_frame(selected_time, self.camera_msgs3_)
                self.msg_flags_[10] = self._find_closest_frame(selected_time, self.camera_msgs4_)
                self.msg_flags_[11] = self._find_closest_frame(selected_time, self.camera_msgs5_)
                self.msg_flags_[12] = self._find_closest_frame(selected_time, self.car_msgs_)
                self.msg_flags_[13] = self._find_closest_frame(selected_time, self.IMU_msgs_)

                # 组装帧并回调发布
                self.packet_callback_msg()

                # clear event → 播放线程阻塞等待下一次 set()
                self.finish_process_event_.clear()

                self.message_callback_(self.frame_msgs_, self.current_frame_, self.msg_flags_)
                rospy.loginfo("Playing frame %d", self.current_frame_)

            # 帧间隔 = 50ms / play_rate（与 C++ 原版一致）
            time.sleep(50 / 1000.0 / self.play_rate_)

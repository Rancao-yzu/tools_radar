#!/usr/bin/env python
import rosbag
import rospy
import threading
import time

class BagReader:
    MAX_TOPIC_NUM = 14

    def __init__(self):
        self.current_frame_ = 0
        self.play_rate_ = 1.0
        self.playing_ = False
        self.main_radar_index_ = 3
        self.message_callback_ = None
        self.update_progress_bar_callback_ = None
        self.play_thread_ = None
        self.mutex_ = threading.Lock()
        self.finish_process_event_ = threading.Event()
        self.finish_process_event_.set()

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
        self.frame_msgs_ = []
        self.msg_flags_ = [-1] * 14

    def read_bag_file(self, file_path):
        if self.update_progress_bar_callback_:
            self.update_progress_bar_callback_(0.05)

        self.stop_bag()

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
        sizes = [
            len(self.pointcloud_msgs0_),
            len(self.pointcloud_msgs1_),
            len(self.pointcloud_msgs2_),
            len(self.pointcloud_msgs3_),
            len(self.pointcloud_msgs4_)
        ]
        return sizes[self.main_radar_index_] if self.main_radar_index_ < 5 else len(self.pointcloud_msgs4_)

    def get_main_radar_time(self, cur_idx):
        msgs_list = [
            self.pointcloud_msgs0_,
            self.pointcloud_msgs1_,
            self.pointcloud_msgs2_,
            self.pointcloud_msgs3_,
            self.pointcloud_msgs4_
        ]
        idx = self.main_radar_index_ if self.main_radar_index_ < 5 else 4
        if 0 <= cur_idx < len(msgs_list[idx]):
            return msgs_list[idx][cur_idx][2]
        return rospy.Time(0)

    def packet_callback_msg(self):
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
        main_radar_size = self.get_main_radar_pcl_size()
        if 0 <= frame_number < main_radar_size:
            self.current_frame_ = frame_number
            if self.message_callback_:
                selected_time = self.get_main_radar_time(self.current_frame_)

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
        if not self.playing_:
            self.playing_ = True
            self.play_thread_ = threading.Thread(target=self._play_loop)
            self.play_thread_.start()

    def stop_bag(self):
        if self.playing_:
            self.playing_ = False
            self.finish_process_event_.set()
            if self.play_thread_ and self.play_thread_.is_alive():
                self.play_thread_.join()

    def set_play_rate(self, rate):
        self.play_rate_ = rate

    def set_message_callback(self, callback):
        self.message_callback_ = callback

    def set_update_progress_bar_callback(self, callback):
        self.update_progress_bar_callback_ = callback

    def get_current_frame(self):
        return self.current_frame_

    def select_main_radar(self, index):
        self.main_radar_index_ = index

    def set_finish_process_flag(self, flag):
        if flag:
            self.finish_process_event_.set()

    def _find_closest_frame(self, selected_time, msg_list):
        if not msg_list:
            return -1
        closest_index = -1
        min_diff = float('inf')
        for i, entry in enumerate(msg_list):
            msg_time = entry[2]
            time_diff = abs((selected_time - msg_time).to_sec())
            if time_diff < min_diff:
                min_diff = time_diff
                closest_index = i
        return closest_index

    def _play_loop(self):
        main_radar_size = self.get_main_radar_pcl_size()
        if self.current_frame_ != 0:
            self.current_frame_ += 1

        for i in range(self.current_frame_, main_radar_size):
            self.finish_process_event_.wait()

            with self.mutex_:
                if not self.playing_:
                    break

                self.current_frame_ = i
                selected_time = self.get_main_radar_time(self.current_frame_)

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
                self.finish_process_event_.clear()
                self.message_callback_(self.frame_msgs_, self.current_frame_, self.msg_flags_)
                rospy.loginfo("Playing frame %d", self.current_frame_)

            time.sleep(50 / 1000.0 / self.play_rate_)

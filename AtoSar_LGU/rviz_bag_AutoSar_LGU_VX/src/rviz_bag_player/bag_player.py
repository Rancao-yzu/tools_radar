#!/usr/bin/env python
# BagPlayer: ROS bag 播放器 PyQt5 GUI，替代原 C++ rviz::Panel 插件
# 功能：选择 .bag 文件 → Read 缓存到内存 → Play/Stop 逐帧播放 → 发布到 ROS 话题
# 逐帧控制：通过 5 个 /play_single_frame_{0..4} 服务，每帧等待客户端确认后才推进
import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLineEdit, QPushButton, QLabel, QSpinBox, QComboBox,
                             QSlider, QProgressBar, QFileDialog)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import CompressedImage
from bag_reader import BagReader


_has_wf_srvs = True
try:
    from wf_srvs.srv import PlaySingleFrame, PlaySingleFrameResponse
except ImportError:
    _has_wf_srvs = False
    print("Warning: wf_srvs not found. PlaySingleFrame services will be disabled.")


_has_arbe_msgs = True
try:
    from arbe_msgs.msg import wfAutosarData, VehStatusOutput, ImuOutput
except ImportError:
    _has_arbe_msgs = False
    print("Warning: arbe_msgs not found. Pointcloud/IMU/Car publishers will be disabled.")


class BagPlayer(QMainWindow):
    """
    ROS bag 播放器主窗口。
    替代原 C++ rviz::Panel 插件，使用 PyQt5 构建独立 GUI。

    --- 信号槽机制 ---
    播放线程在后台运行，不能直接操作 Qt 控件。
    通过 pyqtSignal 跨线程通知主线程更新 UI：
      update_slider_signal  → 更新滑块位置
      update_frame_label_signal → 更新帧信息标签
      update_progress_signal → 更新进度条
    """
    update_slider_signal = pyqtSignal(int)
    update_frame_label_signal = pyqtSignal(str)
    update_progress_signal = pyqtSignal(int)

    def __init__(self):
        super(BagPlayer, self).__init__()
        self.setWindowTitle("ROS Bag Player")
        self.setGeometry(100, 100, 300, 100)   # 初始大小，Qt 会根据内容自动调整

        # ---- 播放器核心 ----
        self.bag_reader_ = BagReader()
        self.frame_count0 = 0   # 前雷达点云帧数
        self.frame_count1 = 0   # 前左角雷达点云帧数
        self.frame_count2 = 0   # 前右角雷达点云帧数
        self.frame_count3 = 0   # 后左角雷达点云帧数
        self.frame_count4 = 0   # 后右角雷达点云帧数
        self.main_radar_index_ = 3   # 当前主雷达索引（默认3=后左角）
        self.b_continue_play_flag = False   # 是否处于连续播放状态，用于防止 play 时响应 spinner/jump
        self.last_path = ""    # 上次读取的路径，防止重复 Read 同一文件

        self.publishers = {}   # ROS Publisher 字典

        self._init_ui()
        self._init_ros()

        # 连接跨线程信号到 UI 更新槽函数
        self.update_slider_signal.connect(self._update_slider)
        self.update_frame_label_signal.connect(self._update_frame_label)
        self.update_progress_signal.connect(self._update_progress)

    def _init_ui(self):
        """构建全部 UI 控件和布局"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # ---- 第一行：文件选择 ----
        self.bag_file_path = QLineEdit()           # bag 文件路径输入框
        self.select_button = QPushButton("Select")  # 浏览选择按钮
        self.read_button = QPushButton("Read")     # 读取缓存按钮
        self.select_button.setFixedSize(45, 30)
        self.read_button.setFixedSize(45, 30)

        file_layout = QHBoxLayout()
        file_layout.addWidget(self.bag_file_path)
        file_layout.addWidget(self.select_button)
        file_layout.addWidget(self.read_button)

        # ---- 第三行：播放控制按钮 ----
        self.step_backward_button = QPushButton("step<-")  # 步进后退
        self.step_forward_button = QPushButton("step->")   # 步进前进
        self.play_button = QPushButton("Play")             # 开始播放
        self.stop_button = QPushButton("Stop")             # 停止播放
        self.play_rate_combo = QComboBox()                 # 播放速率下拉
        self.select_main_radar = QComboBox()               # 主雷达选择下拉

        self.step_backward_button.setFixedSize(50, 30)
        self.step_forward_button.setFixedSize(50, 30)
        self.play_button.setFixedSize(40, 30)
        self.stop_button.setFixedSize(40, 30)

        play_rates = ["1.0", "0.25", "0.5", "1.25", "1.5", "2.0"]
        for rate in play_rates:
            self.play_rate_combo.addItem(rate)

        radar_options = ["前雷达(0)", "前左角(1)", "前右角(2)", "后左角(3)", "后右角(4)", "后雷达(5)"]
        for opt in radar_options:
            self.select_main_radar.addItem(opt)

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.step_backward_button)
        control_layout.addWidget(self.step_forward_button)
        control_layout.addWidget(self.play_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addWidget(QLabel("Rate:"))
        control_layout.addWidget(self.play_rate_combo)
        control_layout.addWidget(self.select_main_radar)

        # ---- 第二行：帧信息 ----
        self.frame_id_label = QLabel("Frame ID: N/A  Timestamp: N/A")
        self.frame_spinner = QSpinBox()           # 帧号微调框
        self.step_spinner = QSpinBox()            # 步进步长（当前隐藏未用）
        self.frame_slider = QSlider(Qt.Horizontal)  # 帧滑块
        self.progress_bar = QProgressBar()          # 读取进度条

        self.frame_spinner.setMinimum(0)
        self.step_spinner.setMinimum(1)
        self.step_spinner.setValue(1)
        self.frame_slider.setMinimum(0)
        self.progress_bar.setRange(0, 120)   # 对应 0.0 ~ 1.2 的进度值
        self.progress_bar.setValue(0)

        self.select_main_radar.setCurrentIndex(3)   # 默认选中后左角雷达

        # ---- 组装布局 ----
        layout.addLayout(file_layout)
        layout.addWidget(self.frame_id_label)
        layout.addWidget(self.frame_spinner)
        layout.addWidget(self.frame_slider)
        layout.addLayout(control_layout)
        layout.addWidget(self.progress_bar)

        # ---- 信号连接 ----
        self.select_button.clicked.connect(self._select_bag_file)
        self.read_button.clicked.connect(self._read_bag_file)
        self.play_button.clicked.connect(self._play_bag)
        self.stop_button.clicked.connect(self._stop_bag)
        self.frame_spinner.valueChanged.connect(self._jump_to_frame)
        self.step_forward_button.clicked.connect(self._step_forward)
        self.step_backward_button.clicked.connect(self._step_backward)
        self.play_rate_combo.currentIndexChanged.connect(self._update_play_rate)
        self.frame_slider.valueChanged.connect(self._slider_value_changed)
        self.select_main_radar.currentIndexChanged.connect(self._select_main_radar)

        # 初始状态：未读取文件前禁用所有播放控件
        self._set_controls_enabled(False)

    def _init_ros(self):
        """
        初始化 ROS 发布者和 PlaySingleFrame 服务。

        --- 服务（5 个）---
        /play_single_frame_0 ~ _4：对应 5 个角雷达
        外部客户端调用 status=1 表示处理完成，触发下一帧播放
        """
        if _has_arbe_msgs:
            self.publishers['pointcloud_pub0'] = rospy.Publisher('/wf/corner_radar/lgu_data_0', wfAutosarData, queue_size=10)
            self.publishers['pointcloud_pub1'] = rospy.Publisher('/wf/corner_radar/lgu_data_1', wfAutosarData, queue_size=10)
            self.publishers['pointcloud_pub2'] = rospy.Publisher('/wf/corner_radar/lgu_data_2', wfAutosarData, queue_size=10)
            self.publishers['pointcloud_pub3'] = rospy.Publisher('/wf/corner_radar/lgu_data_3', wfAutosarData, queue_size=10)
            self.publishers['pointcloud_pub4'] = rospy.Publisher('/wf/corner_radar/lgu_data_4', wfAutosarData, queue_size=10)
            self.publishers['pointcloud_pub5'] = rospy.Publisher('/wf/corner_radar/lgu_data_5', wfAutosarData, queue_size=10)

        self.publishers['camera_pub0'] = rospy.Publisher('/cv_camera_0/image_raw/compressed', CompressedImage, queue_size=10)
        self.publishers['camera_pub1'] = rospy.Publisher('/cv_camera_1/image_raw/compressed', CompressedImage, queue_size=10)
        self.publishers['camera_pub2'] = rospy.Publisher('/cv_camera_2/image_raw/compressed', CompressedImage, queue_size=10)
        self.publishers['camera_pub3'] = rospy.Publisher('/cv_camera_3/image_raw/compressed', CompressedImage, queue_size=10)
        self.publishers['camera_pub4'] = rospy.Publisher('/cv_camera_4/image_raw/compressed', CompressedImage, queue_size=10)
        self.publishers['camera_pub5'] = rospy.Publisher('/cv_camera_5/image_raw/compressed', CompressedImage, queue_size=10)

        if _has_arbe_msgs:
            self.publishers['IMU_pub'] = rospy.Publisher('/wf/imu_data/parsed', ImuOutput, queue_size=10)
            self.publishers['car_pub'] = rospy.Publisher('/wf/car_id6/parsed2', VehStatusOutput, queue_size=1)

        self.publishers['warning_pub'] = rospy.Publisher('/corner_radar/warning_status', UInt8MultiArray, queue_size=10)

        # 创建 5 个 PlaySingleFrame 服务（对应 0~4 号角雷达）
        if _has_wf_srvs:
            for i in range(5):
                rospy.Service(f'/play_single_frame_{i}', PlaySingleFrame, self._handle_play_single_frame)

        # 设置 BagReader 的回调
        self.bag_reader_.set_message_callback(self._on_message_received)
        self.bag_reader_.set_update_progress_bar_callback(self._on_progress_update)

    def _set_controls_enabled(self, enabled):
        """
        统一控制所有播放相关控件的启用/禁用状态。
        读取 bag 时禁用 → 读取完成后启用 → 播放时再次禁用。
        """
        self.play_button.setEnabled(enabled)
        self.stop_button.setEnabled(False)
        self.frame_spinner.setEnabled(enabled)
        self.step_spinner.setEnabled(enabled)
        self.step_forward_button.setEnabled(enabled)
        self.step_backward_button.setEnabled(enabled)
        self.play_rate_combo.setEnabled(enabled)
        self.frame_slider.setEnabled(enabled)
        self.select_main_radar.setEnabled(enabled)

    # ================================================================
    #  UI 控件回调
    # ================================================================

    def _select_bag_file(self):
        """打开文件对话框选择 .bag 文件"""
        file, _ = QFileDialog.getOpenFileName(self, "Select Bag File", "", "Bag Files (*.bag)")
        if file:
            self.bag_file_path.setText(file)

    def _read_bag_file(self):
        """
        读取 bag 文件：
          1. 检查路径是否有变化（相同路径不重复读取）
          2. 禁用控件
          3. 调用 BagReader.read_bag_file() 全量缓存
          4. 更新滑块/微调框范围
          5. 启用控件
        """
        path = self.bag_file_path.text()
        if not path or path == self.last_path:
            return

        self.last_path = path
        self._set_controls_enabled(False)

        try:
            result = self.bag_reader_.read_bag_file(path)
            self.frame_count0, self.frame_count1, self.frame_count2, self.frame_count3, self.frame_count4 = result

            self._update_max_frame()

            self._set_controls_enabled(True)
            rospy.loginfo("Bag file read and cached success: %s", path)
        except Exception as e:
            rospy.logerr("Failed to read bag file: %s", str(e))

    def _update_max_frame(self):
        """
        根据当前主雷达索引，更新滑块和微调框的最大值。
        各雷达点云数量不同，切换雷达时需要同步更新范围。
        """
        max_frame = [self.frame_count0, self.frame_count1, self.frame_count2, self.frame_count3, self.frame_count4]
        if self.main_radar_index_ < 5:
            self.frame_spinner.setMaximum(max_frame[self.main_radar_index_])
            self.frame_slider.setMaximum(max_frame[self.main_radar_index_])
        else:
            self.frame_spinner.setMaximum(self.frame_count4)
            self.frame_slider.setMaximum(self.frame_count4)

    def _jump_to_frame(self):
        """
        通过微调框跳转到指定帧。
        仅在非播放状态（即手动模式）下生效——播放中不响应 spinner 变化。
        """
        if not self.b_continue_play_flag:
            frame_number = self.frame_spinner.value()
            if frame_number >= 0:
                self.bag_reader_.jump_to_frame(frame_number)

    def _step_forward(self):
        """
        步进前进：spinner 值 + step_spinner 值，不超过最大值-1。
        spinner 变化会触发 _jump_to_frame。
        """
        step = self.step_spinner.value()
        new_frame = min(self.frame_spinner.value() + step, self.frame_spinner.maximum() - 1)
        self.frame_spinner.setValue(new_frame)

    def _step_backward(self):
        """
        步进后退：spinner 值 - step_spinner 值，不低于 0。
        """
        step = self.step_spinner.value()
        new_frame = max(self.frame_spinner.value() - step, self.frame_spinner.minimum())
        self.frame_spinner.setValue(new_frame)

    def _play_bag(self):
        """
        开始播放：
          1. 置 b_continue_play_flag = True（停止在 _jump_to_frame 中响应 spinner 变化）
          2. 启动 BagReader 播放线程
          3. 切换按钮状态：Play 禁用、Stop 启用
        """
        self.b_continue_play_flag = True
        self.bag_reader_.play_bag()

        self.play_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.frame_spinner.setEnabled(False)
        self.step_spinner.setEnabled(False)
        self.step_forward_button.setEnabled(False)
        self.step_backward_button.setEnabled(False)
        self.play_rate_combo.setEnabled(False)
        self.frame_slider.setEnabled(False)
        self.select_main_radar.setEnabled(False)

    def _stop_bag(self):
        """
        停止播放：
          1. 置 b_continue_play_flag = False
          2. 调用 BagReader.stop_bag()（set event → join 线程）
          3. 恢复控件状态
        """
        self.b_continue_play_flag = False
        self.bag_reader_.stop_bag()

        self.play_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.frame_spinner.setEnabled(True)
        self.step_spinner.setEnabled(True)
        self.step_forward_button.setEnabled(True)
        self.step_backward_button.setEnabled(True)
        self.play_rate_combo.setEnabled(True)
        self.frame_slider.setEnabled(True)
        self.select_main_radar.setEnabled(True)

    def _update_play_rate(self):
        """播放速率下拉变化时更新 BagReader 的 play_rate_"""
        rate = float(self.play_rate_combo.currentText())
        self.bag_reader_.set_play_rate(rate)

    def _slider_value_changed(self, value):
        """
        帧滑块值变化时，同步更新微调框。
        到达最大值时回绕到 0（循环）。
        """
        if self.frame_spinner.value() >= self.frame_spinner.maximum() or value >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)
        else:
            self.frame_spinner.setValue(value)

    def _select_main_radar(self):
        """
        切换主雷达下拉时，更新 main_radar_index_ 并同步 BagReader 和 UI 范围。
        如果当前帧超出新雷达的范围，复位到 0。
        """
        self.main_radar_index_ = self.select_main_radar.currentIndex()
        self.bag_reader_.select_main_radar(self.main_radar_index_)
        self._update_max_frame()

        if self.frame_spinner.value() >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)

    # ================================================================
    #  ROS 服务回调
    # ================================================================

    def _handle_play_single_frame(self, req):
        """
        PlaySingleFrame 服务回调。
        5 个角雷达各自一个服务 /play_single_frame_{0..4}。

        请求字段:
          radar_pos: uint8  雷达编号 (0~4)
          frame_id:  uint16 帧 ID
          status:    uint8  0=收到数据, 1=处理完成请求下一帧, 其他=错误

        响应字段:
          success: bool

        与 C++ 原版 handleServiceRequest 逻辑完全一致：
          - 仅响应匹配当前 main_radar_index_ 的请求
          - status==1 时调用 set_finish_process_flag(True) 唤醒播放线程
        """
        res = PlaySingleFrameResponse()
        if req.radar_pos == self.main_radar_index_:   # 仅处理匹配当前主雷达的请求
            if req.status == 0:
                rospy.loginfo("Received data: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id)
            elif req.status == 1:
                rospy.loginfo("Finish process: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id)
                self.bag_reader_.set_finish_process_flag(True)   # set event → 播放线程继续下一帧
            else:
                rospy.loginfo("Play single frame service error!")
            res.success = True
        return res

    # ================================================================
    #  BagReader 回调（在播放线程中调用，通过信号转发到主线程更新 UI）
    # ================================================================

    def _on_message_received(self, frame_msg, frame_number, msg_flag):
        """
        BagReader 消息回调（在播放线程中执行）。

        步骤：
          1. 发布 frame_msg 中各话题的消息到对应 ROS 话题
          2. 发射信号通知主线程更新滑块位置
          3. 取主雷达点云消息（index=3）的 frameID 和 timestamp 更新标签
        """
        self._publish_messages(frame_msg, msg_flag)
        self.update_slider_signal.emit(self.bag_reader_.get_current_frame())

        # 显示主雷达（索引3）的帧信息
        if len(frame_msg) > 3 and msg_flag[3] >= 0:
            entry = frame_msg[3]
            if entry is not None:
                try:
                    data = entry[1]   # entry = (topic, msg, time)
                    frame_id = data.frameID if hasattr(data, 'frameID') else "N/A"
                    timestamp = str(data.header.stamp) if hasattr(data, 'header') else "N/A"
                    self.update_frame_label_signal.emit(f"-Frame ID: {frame_id}  -Timestamp: {timestamp}")
                except Exception:
                    pass

    def _publish_messages(self, frame_msg, msg_flag):
        """
        遍历 frame_msg 的 14 个槽位，将有数据的消息发布到对应 ROS 话题。

        frame_msg 索引对应关系：
          [0]~[4]   点云 (需要 arbe_msgs)
          [5]       warning 状态
          [6]~[11]  相机图像
          [12]      车辆状态 (需要 arbe_msgs)
          [13]      IMU (需要 arbe_msgs)

        仅当 msg_flag[i] >= 0（该话题有匹配消息）时才发布。
        """
        # 车辆状态 [12]
        if _has_arbe_msgs and len(frame_msg) > 12 and msg_flag[12] >= 0:
            self._safe_publish('car_pub', frame_msg[12])

        # 点云 [0]~[4]
        for i in range(5):
            if _has_arbe_msgs and len(frame_msg) > i and msg_flag[i] >= 0:
                self._safe_publish(f'pointcloud_pub{i}', frame_msg[i])

        # 相机 [6]~[11]
        for i in range(6):
            idx = i + 6
            if len(frame_msg) > idx and msg_flag[idx] >= 0:
                self._safe_publish(f'camera_pub{i}', frame_msg[idx])

        # IMU [13]
        if _has_arbe_msgs and len(frame_msg) > 13 and msg_flag[13] >= 0:
            self._safe_publish('IMU_pub', frame_msg[13])

        # 警告 [5]
        if len(frame_msg) > 5 and msg_flag[5] >= 0:
            self._safe_publish('warning_pub', frame_msg[5])

    def _safe_publish(self, pub_key, entry):
        """
        安全发布消息：忽略 None 条目，捕获所有异常不中断播放。
        entry 是 (topic, msg, time) 三元组，entry[1] 是 ROS 消息体。
        """
        if entry is None:
            return
        try:
            self.publishers[pub_key].publish(entry[1])
        except Exception as e:
            rospy.loginfo("%s publish error: %s", pub_key, str(e))

    def _on_progress_update(self, value):
        """
        BagReader 读取进度回调。
        value 范围 0.0 ~ 1.2，映射到进度条的 0~120。
        """
        self.update_progress_signal.emit(int(value * 100))

    # ================================================================
    #  Qt 槽函数（在主线程中执行，更新 UI）
    # ================================================================

    @pyqtSlot(int)
    def _update_slider(self, value):
        """更新帧滑块位置（由播放线程的 update_slider_signal 触发）"""
        self.frame_slider.setValue(value)

    @pyqtSlot(str)
    def _update_frame_label(self, text):
        """更新帧信息标签"""
        self.frame_id_label.setText(text)

    @pyqtSlot(int)
    def _update_progress(self, value):
        """更新进度条"""
        self.progress_bar.setValue(value)

    def closeEvent(self, event):
        """窗口关闭时停止播放并接受关闭事件"""
        self._stop_bag()
        event.accept()


def main():
    """
    入口函数。

    启动流程：
      1. rospy.init_node() — 必须在主线程中调用（ROS Noetic 信号处理器限制）
      2. 创建 QApplication
      3. 创建 10ms QTimer 空轮询 — 驱动 Qt 事件循环以处理 rospy 的 service/topic 回调
      4. 创建 BagPlayer 窗口并显示
      5. 进入 Qt 事件循环
    """
    rospy.init_node('bag_player_node', anonymous=True)

    app = QApplication(sys.argv)

    # QTimer 定期触发空回调，让 Qt 事件循环保持活跃
    # rospy 的 Service 和 Topic 回调依赖 Qt 事件循环来分派
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: None)
    ros_timer.start(10)

    window = BagPlayer()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

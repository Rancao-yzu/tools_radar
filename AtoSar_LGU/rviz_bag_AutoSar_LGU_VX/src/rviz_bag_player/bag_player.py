#!/usr/bin/env python
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
    update_slider_signal = pyqtSignal(int)
    update_frame_label_signal = pyqtSignal(str)
    update_progress_signal = pyqtSignal(int)

    def __init__(self):
        super(BagPlayer, self).__init__()
        self.setWindowTitle("ROS Bag Player")
        self.setGeometry(100, 100, 400, 200)

        self.bag_reader_ = BagReader()
        self.frame_count0 = 0
        self.frame_count1 = 0
        self.frame_count2 = 0
        self.frame_count3 = 0
        self.frame_count4 = 0
        self.main_radar_index_ = 3
        self.b_continue_play_flag = False
        self.last_path = ""

        self.publishers = {}

        self._init_ui()
        self._init_ros()

        self.update_slider_signal.connect(self._update_slider)
        self.update_frame_label_signal.connect(self._update_frame_label)
        self.update_progress_signal.connect(self._update_progress)

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.bag_file_path = QLineEdit()
        self.select_button = QPushButton("Select")
        self.read_button = QPushButton("Read")
        self.select_button.setFixedSize(45, 30)
        self.read_button.setFixedSize(45, 30)

        file_layout = QHBoxLayout()
        file_layout.addWidget(self.bag_file_path)
        file_layout.addWidget(self.select_button)
        file_layout.addWidget(self.read_button)

        self.step_backward_button = QPushButton("step<-")
        self.step_forward_button = QPushButton("step->")
        self.play_button = QPushButton("Play")
        self.stop_button = QPushButton("Stop")
        self.play_rate_combo = QComboBox()
        self.select_main_radar = QComboBox()

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
        control_layout.addWidget(QLabel("Play Rate:"))
        control_layout.addWidget(self.play_rate_combo)
        control_layout.addWidget(self.select_main_radar)

        self.frame_id_label = QLabel("Frame ID: N/A  Timestamp: N/A")
        self.frame_spinner = QSpinBox()
        self.step_spinner = QSpinBox()
        self.frame_slider = QSlider(Qt.Horizontal)
        self.progress_bar = QProgressBar()

        self.frame_spinner.setMinimum(0)
        self.step_spinner.setMinimum(1)
        self.step_spinner.setValue(1)
        self.frame_slider.setMinimum(0)
        self.progress_bar.setRange(0, 120)
        self.progress_bar.setValue(0)

        self.select_main_radar.setCurrentIndex(3)

        layout.addLayout(file_layout)
        layout.addWidget(self.frame_id_label)
        layout.addWidget(self.frame_spinner)
        layout.addWidget(self.frame_slider)
        layout.addLayout(control_layout)
        layout.addWidget(self.progress_bar)

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

        self._set_controls_enabled(False)

    def _init_ros(self):
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

        if _has_wf_srvs:
            for i in range(5):
                rospy.Service(f'/play_single_frame_{i}', PlaySingleFrame, self._handle_play_single_frame)

        self.bag_reader_.set_message_callback(self._on_message_received)
        self.bag_reader_.set_update_progress_bar_callback(self._on_progress_update)

    def _set_controls_enabled(self, enabled):
        self.play_button.setEnabled(enabled)
        self.stop_button.setEnabled(False)
        self.frame_spinner.setEnabled(enabled)
        self.step_spinner.setEnabled(enabled)
        self.step_forward_button.setEnabled(enabled)
        self.step_backward_button.setEnabled(enabled)
        self.play_rate_combo.setEnabled(enabled)
        self.frame_slider.setEnabled(enabled)
        self.select_main_radar.setEnabled(enabled)

    def _select_bag_file(self):
        file, _ = QFileDialog.getOpenFileName(self, "Select Bag File", "", "Bag Files (*.bag)")
        if file:
            self.bag_file_path.setText(file)

    def _read_bag_file(self):
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
        max_frame = [self.frame_count0, self.frame_count1, self.frame_count2, self.frame_count3, self.frame_count4]
        if self.main_radar_index_ < 5:
            self.frame_spinner.setMaximum(max_frame[self.main_radar_index_])
            self.frame_slider.setMaximum(max_frame[self.main_radar_index_])
        else:
            self.frame_spinner.setMaximum(self.frame_count4)
            self.frame_slider.setMaximum(self.frame_count4)

    def _jump_to_frame(self):
        if not self.b_continue_play_flag:
            frame_number = self.frame_spinner.value()
            if frame_number >= 0:
                self.bag_reader_.jump_to_frame(frame_number)

    def _step_forward(self):
        step = self.step_spinner.value()
        new_frame = min(self.frame_spinner.value() + step, self.frame_spinner.maximum() - 1)
        self.frame_spinner.setValue(new_frame)

    def _step_backward(self):
        step = self.step_spinner.value()
        new_frame = max(self.frame_spinner.value() - step, self.frame_spinner.minimum())
        self.frame_spinner.setValue(new_frame)

    def _play_bag(self):
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
        rate = float(self.play_rate_combo.currentText())
        self.bag_reader_.set_play_rate(rate)

    def _slider_value_changed(self, value):
        if self.frame_spinner.value() >= self.frame_spinner.maximum() or value >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)
        else:
            self.frame_spinner.setValue(value)

    def _select_main_radar(self):
        self.main_radar_index_ = self.select_main_radar.currentIndex()
        self.bag_reader_.select_main_radar(self.main_radar_index_)
        self._update_max_frame()

        if self.frame_spinner.value() >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)

    def _handle_play_single_frame(self, req):
        res = PlaySingleFrameResponse()
        if req.radar_pos == self.main_radar_index_:
            if req.status == 0:
                rospy.loginfo("Received data: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id)
            elif req.status == 1:
                rospy.loginfo("Finish process: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id)
                self.bag_reader_.set_finish_process_flag(True)
            else:
                rospy.loginfo("Play single frame service error!")
            res.success = True
        return res

    def _on_message_received(self, frame_msg, frame_number, msg_flag):
        self._publish_messages(frame_msg, msg_flag)
        self.update_slider_signal.emit(self.bag_reader_.get_current_frame())

        if len(frame_msg) > 3 and msg_flag[3] >= 0:
            entry = frame_msg[3]
            if entry is not None:
                try:
                    data = entry[1]
                    frame_id = data.frameID if hasattr(data, 'frameID') else "N/A"
                    timestamp = str(data.header.stamp) if hasattr(data, 'header') else "N/A"
                    self.update_frame_label_signal.emit(f"-Frame ID: {frame_id}  -Timestamp: {timestamp}")
                except Exception:
                    pass

    def _publish_messages(self, frame_msg, msg_flag):
        if _has_arbe_msgs and len(frame_msg) > 12 and msg_flag[12] >= 0:
            self._safe_publish('car_pub', frame_msg[12])

        for i in range(5):
            if _has_arbe_msgs and len(frame_msg) > i and msg_flag[i] >= 0:
                self._safe_publish(f'pointcloud_pub{i}', frame_msg[i])

        for i in range(6):
            idx = i + 6
            if len(frame_msg) > idx and msg_flag[idx] >= 0:
                self._safe_publish(f'camera_pub{i}', frame_msg[idx])

        if _has_arbe_msgs and len(frame_msg) > 13 and msg_flag[13] >= 0:
            self._safe_publish('IMU_pub', frame_msg[13])

        if len(frame_msg) > 5 and msg_flag[5] >= 0:
            self._safe_publish('warning_pub', frame_msg[5])

    def _safe_publish(self, pub_key, entry):
        if entry is None:
            return
        try:
            self.publishers[pub_key].publish(entry[1])
        except Exception as e:
            rospy.loginfo("%s publish error: %s", pub_key, str(e))

    def _on_progress_update(self, value):
        self.update_progress_signal.emit(int(value * 100))

    @pyqtSlot(int)
    def _update_slider(self, value):
        self.frame_slider.setValue(value)

    @pyqtSlot(str)
    def _update_frame_label(self, text):
        self.frame_id_label.setText(text)

    @pyqtSlot(int)
    def _update_progress(self, value):
        self.progress_bar.setValue(value)

    def closeEvent(self, event):
        self._stop_bag()
        event.accept()


def main():
    rospy.init_node('bag_player_node', anonymous=True)

    app = QApplication(sys.argv)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: None)
    ros_timer.start(10)

    window = BagPlayer()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

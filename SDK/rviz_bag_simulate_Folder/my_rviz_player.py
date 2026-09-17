#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前端：PyQt5 界面。播放/读取/发布等逻辑在 bag_player_core.py（前后端通过回调通信）。

消息定义在 src/（标准 catkin 工作空间，仅含 arbe_msgs / wf_srvs 两个消息包），
catkin_make 后生成的 Python 模块位于 devel/，启动时自动加入 sys.path。
改消息：编辑 src/ 下的 .msg / .srv 后重新运行 ./start.sh（会自动重建）。

单个 bag 与文件夹共用 Play：播哪个取决于最后选择的路径（选择一方时会清空另一方）。
"""

import os
import sys
import signal

# 使用本工作空间生成的消息模块，必须在 import bag_player_core 之前加入路径
_PKGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'devel', 'lib', 'python3', 'dist-packages')
if not os.path.isdir(os.path.join(_PKGS, 'arbe_msgs')):
    sys.exit('错误：消息模块未生成。请先运行 ./start.sh 或 '
             'catkin_make --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5')
sys.path.insert(0, _PKGS)

import rospy

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
    QLabel, QSpinBox, QComboBox, QCheckBox, QSlider, QProgressBar, QFileDialog)

from bag_player_core import BagPlayer


class PlayerWindow(QWidget):
    # 后端回调桥接为 Qt 信号（工作线程 emit 自动排队到 UI 线程）
    progressUpdated = pyqtSignal(float)
    frameUpdated = pyqtSignal(int)
    frameInfoUpdated = pyqtSignal(str)
    playFinished = pyqtSignal(int)
    bagLoadedSig = pyqtSignal(object)

    def __init__(self, parent=None):
        super(PlayerWindow, self).__init__(parent)
        self.setWindowTitle('Bag Player')
        self._loading = False
        self._folder_files = None   # 文件夹模式文件列表（None = 单个 bag 模式）
        self._folder_index = 0       # 当前已加载的文件索引
        self._current_name = ''      # 当前已加载的 bag 文件名

        # ---------------- 后端 ----------------
        self.player = BagPlayer()
        self.player.on_progress = self.progressUpdated.emit
        self.player.on_frame = self.frameUpdated.emit
        self.player.on_frame_info = self.frameInfoUpdated.emit
        self.player.on_play_finished = self.playFinished.emit
        self.player.on_bag_loaded = self.bagLoadedSig.emit

        # ---------------- UI ----------------
        self.bag_file_path = QLineEdit()
        self.select_button = QPushButton('Select')
        self.read_button = QPushButton('Read')
        self.folder_path = QLineEdit()
        self.select_folder_button = QPushButton('Folder')
        self.folder_read_button = QPushButton('Read')
        self.play_button = QPushButton('Play')
        self.stop_button = QPushButton('Stop')
        self.step_forward_button = QPushButton('->')
        self.step_backward_button = QPushButton('<-')
        self.frame_spinner = QSpinBox()
        self.step_spinner = QSpinBox()
        self.frame_count_label = QLabel('Frame : Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0')
        self.frame_id_label = QLabel('Frame ID: N/A  Timestamp: N/A')
        self.status_label = QLabel('')
        self.play_rate_combo = QComboBox()
        self.play_sp_date = QCheckBox('SP')
        self.select_main_radar = QComboBox()
        self.frame_slider = QSlider(Qt.Horizontal)
        self.progress_bar = QProgressBar()

        for r in ('1.0', '0.25', '0.5', '1.25', '1.5', '2.0'):
            self.play_rate_combo.addItem(r)

        self.select_button.setFixedSize(40, 30)
        self.select_folder_button.setFixedSize(40, 30)
        self.folder_read_button.setFixedSize(40, 30)
        self.read_button.setFixedSize(40, 30)
        self.play_button.setFixedSize(40, 30)
        self.stop_button.setFixedSize(40, 30)
        self.step_forward_button.setFixedSize(25, 30)
        self.step_backward_button.setFixedSize(25, 30)

        self.select_main_radar.addItem('前(0)')
        self.select_main_radar.addItem('前左(1)')
        self.select_main_radar.addItem('前右(2)')
        self.select_main_radar.addItem('后左(3)')
        self.select_main_radar.addItem('后右(4)')
        self.select_main_radar.addItem('后(5)')

        self.frame_spinner.setMinimum(0)
        self.step_spinner.setMinimum(1)
        self.step_spinner.setValue(1)
        self.frame_slider.setMinimum(0)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)

        layout = QVBoxLayout()
        file_layout = QHBoxLayout()
        file_layout.addWidget(self.bag_file_path)
        file_layout.addWidget(self.select_button)
        file_layout.addWidget(self.read_button)
        folder_layout = QHBoxLayout()
        folder_layout.addWidget(self.folder_path)
        folder_layout.addWidget(self.select_folder_button)
        folder_layout.addWidget(self.folder_read_button)
        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel('Step:'))
        control_layout.addWidget(self.step_spinner)
        control_layout.addWidget(self.step_backward_button)
        control_layout.addWidget(self.step_forward_button)
        control_layout.addWidget(self.play_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addWidget(QLabel('Rate:'))
        control_layout.addWidget(self.play_rate_combo)
        control_layout.addWidget(self.play_sp_date)
        control_layout.addWidget(self.select_main_radar)
        layout.addLayout(file_layout)
        layout.addLayout(folder_layout)
        layout.addWidget(self.frame_count_label)
        layout.addWidget(self.frame_id_label)
        layout.addWidget(self.status_label)
        layout.addWidget(self.frame_spinner)
        layout.addWidget(self.frame_slider)
        layout.addLayout(control_layout)
        layout.addWidget(self.progress_bar)
        self.setLayout(layout)

        self.select_button.clicked.connect(self.select_bag_file)
        self.read_button.clicked.connect(self.read_bag_file)
        self.play_button.clicked.connect(self.play_bag)
        self.stop_button.clicked.connect(self.stop_bag)
        self.frame_spinner.valueChanged.connect(self.jump_to_frame)
        self.step_forward_button.clicked.connect(self.step_forward)
        self.step_backward_button.clicked.connect(self.step_backward)
        self.play_rate_combo.currentIndexChanged.connect(self.update_play_rate)
        self.frame_slider.valueChanged.connect(self.slider_value_changed)
        self.play_sp_date.stateChanged.connect(self.set_play_sp_flag)
        self.select_main_radar.currentIndexChanged.connect(self.select_main_radar_slot)
        self.select_folder_button.clicked.connect(self.select_folder)
        self.folder_read_button.clicked.connect(self.read_folder)

        self.progressUpdated.connect(self._on_progress)
        self.frameUpdated.connect(self._on_frame_updated)
        self.frameInfoUpdated.connect(self.frame_id_label.setText)
        self.playFinished.connect(self._on_play_finished)
        self.bagLoadedSig.connect(self._on_bag_loaded)

        self._set_controls('init')
        self.select_main_radar.setCurrentIndex(self.player.main_radar_index)

    # ================= UI 状态 =================
    def _set_controls(self, mode):
        not_busy = False if mode in ('loading', 'playing') else True
        self.play_button.setEnabled(mode == 'ready')
        self.stop_button.setEnabled(mode == 'playing')
        for w in (self.frame_spinner, self.step_spinner, self.step_forward_button,
                  self.step_backward_button, self.play_rate_combo, self.frame_slider,
                  self.play_sp_date, self.select_main_radar):
            w.setEnabled(mode == 'ready')
        # 读取 bag / 播放过程中禁用所有入口，避免中途点击干扰
        self.read_button.setEnabled(not_busy)
        self.select_button.setEnabled(not_busy)
        self.select_folder_button.setEnabled(not_busy)
        self.folder_read_button.setEnabled(not_busy)

    def _update_frame_ranges(self):
        n = self.player.main_count()
        self.frame_spinner.setMaximum(n)
        self.frame_slider.setMaximum(n)
        if self.frame_spinner.value() >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)

    # ================= 槽：文件选择 / 读取 =================
    def select_bag_file(self):
        file, _ = QFileDialog.getOpenFileName(self, 'Select Bag File', '', 'Bag Files (*.bag)')
        if not file:
            return
        self.player.shutdown()   # 切到单个模式，先停掉正在进行的播放
        QApplication.processEvents()
        self.bag_file_path.setText(file)
        self.folder_path.clear()   # 最后选择的是单个文件，清空文件夹路径
        self._folder_files = None
        self._set_controls('init')   # 必须先 Read

    def select_folder(self):
        d = QFileDialog.getExistingDirectory(self, 'Select Bag Folder', '')
        if not d:
            return
        self.player.shutdown()   # 切到文件夹模式，先停掉正在进行的播放
        QApplication.processEvents()
        self._folder_files = None   # 重新选择，需重新 FolderRead
        self.folder_path.setText(d)
        self.bag_file_path.clear()   # 最后选择的是文件夹，清空单个 bag 路径
        self._set_controls('init')   # 必须先 FolderRead

    def read_folder(self):
        d = self.folder_path.text()
        if not d:
            return
        files = sorted(os.path.join(d, f) for f in os.listdir(d) if f.endswith('.bag'))
        if not files:
            rospy.logwarn('No .bag files in folder: %s', d)
            return
        self._folder_files = files
        self._folder_index = 0
        self._load_bag_sync(files[0])   # 只读第一个，播完后在 _on_play_finished 里逐个读下一个

    def read_bag_file(self):
        path = self.bag_file_path.text()
        if not path:
            return
        self._folder_files = None   # 切回单个模式
        self._load_bag_sync(path)

    def _load_bag_sync(self, path):
        """UI 线程同步加载一个 bag（进度回调里 processEvents 刷新界面）"""
        self._loading = True
        self._set_controls('loading')
        self.status_label.setText('Loading: %s' % os.path.basename(path))
        self.player.shutdown()
        QApplication.processEvents()   # 消化迟到的 playFinished（被 _loading 拦截）
        self.player.load_bag(path)
        self._loading = False

    def _on_progress(self, v):
        self.progress_bar.setValue(int(v * 100))
        QApplication.processEvents()   # 让进度条在读取过程中刷新

    def _on_bag_loaded(self, d):
        self.frame_count_label.setText('Frame:(0)-%d; (1)-%d; (2)-%d; (3)-%d; (4)-%d' % tuple(d['pcl']))
        # 屏蔽信号复位，避免触发 jump_to_frame 在 read 阶段发布首帧（与 C++ 一致）
        self.frame_spinner.blockSignals(True)
        self.frame_slider.blockSignals(True)
        self.frame_spinner.setValue(0)
        self.frame_slider.setValue(0)
        self.frame_spinner.blockSignals(False)
        self.frame_slider.blockSignals(False)
        self._update_frame_ranges()
        self._current_name = os.path.basename(d['path'])
        if self._folder_files is not None:
            self.status_label.setText('Folder [%d/%d]: %s'
                                      % (self._folder_index + 1, len(self._folder_files), self._current_name))
        else:
            self.status_label.setText(self._current_name)
        if d.get('enable_ui', False):
            self._set_controls('ready')
        rospy.loginfo('----Bag file read and cached success: %s', d['path'])

    # ================= 槽：跳转 / 单步 =================
    def jump_to_frame(self):
        if not self.player.playing:
            self.player.jump(self.frame_spinner.value())

    def step_forward(self):
        step = self.step_spinner.value()
        self.frame_spinner.setValue(min(self.frame_spinner.value() + step, self.frame_spinner.maximum() - 1))

    def step_backward(self):
        step = self.step_spinner.value()
        self.frame_spinner.setValue(max(self.frame_spinner.value() - step, self.frame_spinner.minimum()))

    def _on_frame_updated(self, frame):
        self.frame_slider.blockSignals(True)
        self.frame_spinner.blockSignals(True)
        self.frame_slider.setValue(frame)
        self.frame_spinner.setValue(frame)
        self.frame_slider.blockSignals(False)
        self.frame_spinner.blockSignals(False)

    def slider_value_changed(self, value):
        if self.frame_spinner.value() >= self.frame_spinner.maximum() or value >= self.frame_spinner.maximum():
            self.frame_spinner.setValue(0)
            self.frame_slider.setValue(0)
        else:
            self.frame_spinner.setValue(value)

    def update_play_rate(self):
        try:
            self.player.play_rate = float(self.play_rate_combo.currentText())
        except ValueError:
            pass

    def set_play_sp_flag(self, flag):
        self.player.sp_flag = bool(flag)
        self._update_frame_ranges()

    def select_main_radar_slot(self, index):
        self.player.main_radar_index = index
        self._update_frame_ranges()

    # ================= 槽：播放 =================
    def play_bag(self):
        # 统一只播"当前已加载的内容"。先确保旧播放线程退出再启动新播放；
        # processEvents 放在 play() 之后，配合 play_serial 丢弃迟到的旧 playFinished。
        self.player.shutdown()
        self.player.play()
        self._set_controls('playing')
        if self._folder_files is not None:
            self.status_label.setText('Folder [%d/%d]: %s'
                                      % (self._folder_index + 1, len(self._folder_files), self._current_name))
        else:
            self.status_label.setText(self._current_name)
        QApplication.processEvents()

    def stop_bag(self):
        self.player.stop()

    def _on_play_finished(self, serial):
        if serial != self.player.play_serial:
            return   # 迟到的旧播放结束信号，丢弃
        if self._loading:
            return   # 正在读取新 bag，控件由读取流程恢复
        if not self.player.stop_requested:
            # 自然播完：文件夹模式还有下一个时，读下一个接着播
            if self._folder_files is not None and self._folder_index + 1 < len(self._folder_files):
                self._folder_index += 1
                self._load_bag_sync(self._folder_files[self._folder_index])
                self.player.play()
                self._set_controls('playing')
                return
            self.status_label.setText('Folder play finished' if self._folder_files is not None else 'Play finished')
            
        self._set_controls('ready')

    # ================= 退出清理 =================
    def closeEvent(self, event):
        self.player.shutdown()
        event.accept()


def main():
    rospy.init_node('my_rviz_player_py')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    win = PlayerWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
角雷达时间戳提取工具 GUI
自动提取文件夹内所有bag文件的角雷达ADAS报警时间戳
"""

import sys
import os
import csv
import re
from pathlib import Path
from dataclasses import dataclass
from threading import Thread
import rosbag
from std_msgs.msg import Int32MultiArray

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# 功能名称映射表
# 索引0为None，索引1-15对应15个功能
FUNC_NAMES = [
    None, "LeftBsd", "RightBsd", "LeftLca", "RightLca", "LeftDow", "RightDow",
    "Rcw", "LeftRcta", "RightRcta", "LeftRctb", "RightRctb", "LeftFcta", "RightFcta", "LeftFctb", "RightFctb"
]

def extract_date_from_filename(filename):
    """
    从文件名中提取年月日(格式:YYYYMMDD)
    支持格式: YYYY-MM-DD, YYYY_MM_DD, YYYYMMDD
    """
    # 匹配日期模式
    patterns = [
        r'(\d{4})[-_](\d{2})[-_](\d{2})',  # YYYY-MM-DD 或 YYYY_MM_DD
        r'(\d{4})(\d{2})(\d{2})',          # YYYYMMDD
    ]
    
    for pattern in patterns:
        match = re.search(pattern, filename)
        if match:
            year, month, day = match.groups()
            return f"{year}{month}{day}"  # 返回YYYYMMDD格式
    
    return None  # 未找到日期

@dataclass
class Event:
    """报警事件类，表示一个连续的报警区间"""
    start_t: float       # 开始时间戳(秒)
    end_t: float         # 结束时间戳(秒)
    start_idx: int       # 开始帧号
    end_idx: int         # 结束帧号
    radar_id: int        # 雷达ID
    func_idx: int        # 功能索引(1-15)

    def duration(self):
        """计算持续时间(秒)"""
        return max(0.0, self.end_t - self.start_t)
    
    def func_name(self):
        """获取功能名称"""
        return FUNC_NAMES[self.func_idx] or f"Func{self.func_idx}"
    
    def to_dict(self, bag_name, source):
        """转换为字典格式，用于CSV输出"""
        return {
            "bag": bag_name,
            "radar_id": self.radar_id,
            "warning_type": self.func_name(),
            "func_idx": self.func_idx,
            "source": source,  # "GT"或"WF"
            "start_time": self.start_t,
            "end_time": self.end_t,
            "duration_sec": self.duration(),
            "start_frame": self.start_idx,
            "end_frame": self.end_idx
        }


def read_radar_samples(bag, topic):
    """
    读取雷达数据
    参数:
        bag: rosbag.Bag对象
        topic: 要读取的topic名称
    返回:
        字典结构: {雷达ID: {功能索引: [(时间戳, 位值, 帧号), ...]}}
    """
    samples = {}  # 存储样本数据
    frame_idx = -1  # 帧号计数器
    
    # 遍历topic中的所有消息
    for _, msg, t in bag.read_messages(topics=[topic]):
        # 提取数据
        data = msg.data if isinstance(msg, Int32MultiArray) else getattr(msg, "data", None)
        if not data or len(data) < 16:
            continue  # 跳过无效数据
        
        # 解析雷达ID和数据
        radar_id = int(data[0])  # 第一个元素是雷达ID
        frame_idx += 1  # 帧号递增
        ts = t.to_sec()  # 转换为秒

         # 计算调整后的帧号（对齐到参考帧率）
        adjusted_frame_idx = int(frame_idx / 2.65)  # 取整数
        
        # 初始化数据结构
        if radar_id not in samples:
            # 为每个雷达创建15个功能的存储列表
            samples[radar_id] = {i: [] for i in range(1, 16)}
        
        # 提取每个功能的位值(0或1)
        for func_idx in range(1, 16):
            bit = 1 if int(data[func_idx]) != 0 else 0
            # 存储(时间戳, 位值, 帧号)
            samples[radar_id][func_idx].append((ts, bit, adjusted_frame_idx))
    
    return samples


def samples_to_events(samples, radar_id, func_idx):
    """
    将连续的样本数据转换为报警事件
    算法: 检测位值从0到1(开始)和从1到0(结束)的变化
    """
    events = []
    if not samples:
        return events  # 无样本数据
    
    # 按时间戳排序
    sorted_samples = sorted(samples, key=lambda x: x[0])
    
    # 初始化变量
    prev_t, prev_b, prev_idx = sorted_samples[0]  # 前一个样本
    high_start_t = None  # 报警开始时间
    high_start_idx = None  # 报警开始帧号
    
    # 检查第一个样本是否为高电平(报警开始)
    if prev_b == 1:
        high_start_t = prev_t
        high_start_idx = prev_idx
    
    # 遍历后续样本
    for t, b, idx in sorted_samples[1:]:
        if b == prev_b:  # 状态未变化
            prev_t, prev_b, prev_idx = t, b, idx
            continue
        
        # 状态变化处理
        if prev_b == 0 and b == 1:  # 上升沿: 报警开始
            high_start_t = t
            high_start_idx = idx
        elif prev_b == 1 and b == 0:  # 下降沿: 报警结束
            if high_start_t is not None:  # 确保有开始记录
                events.append(Event(high_start_t, t, high_start_idx, idx, radar_id, func_idx))
                high_start_t = None  # 重置开始标记
        
        # 更新前一个样本
        prev_t, prev_b, prev_idx = t, b, idx
    
    # 处理末尾仍为高电平的情况
    last_t, last_b, last_idx = sorted_samples[-1]
    if last_b == 1 and high_start_t is not None:
        events.append(Event(high_start_t, last_t, high_start_idx, last_idx, radar_id, func_idx))
    
    return events


def extract_timestamps_from_bag(bag_path):
    """
    从单个bag文件提取时间戳
    """
    all_results = []
    bag_name = os.path.basename(bag_path)  # 提取文件名
    
    try:
        # 打开bag文件
        bag = rosbag.Bag(bag_path, "r")
        
        # 读取GT数据(真值)
        gt_samples = read_radar_samples(bag, "/corner_radar/sil/warning_status")
        
        # 读取WF数据(工作流)
        wf_samples = read_radar_samples(bag, "/corner_radar/warning_status")
        
        # 合并所有雷达ID
        all_radar_ids = set(gt_samples.keys()) | set(wf_samples.keys())
        
        # 处理每个雷达的数据
        for radar_id in sorted(all_radar_ids):
            # 处理GT数据
            if radar_id in gt_samples:
                for func_idx in range(1, 16):
                    samples = gt_samples[radar_id].get(func_idx, [])
                    events = samples_to_events(samples, radar_id, func_idx)
                    for ev in events:
                        all_results.append(ev.to_dict(bag_name, "GT"))
            
            # 处理WF数据
            if radar_id in wf_samples:
                for func_idx in range(1, 16):
                    samples = wf_samples[radar_id].get(func_idx, [])
                    events = samples_to_events(samples, radar_id, func_idx)
                    for ev in events:
                        all_results.append(ev.to_dict(bag_name, "WF"))
        
        return all_results
        
    except Exception as e:
        raise Exception(f"处理文件 {bag_name} 失败: {str(e)}")
    finally:
        if 'bag' in locals():
            bag.close()  # 确保关闭bag文件


class WorkerSignals(QObject):
    """工作线程信号类，用于线程间通信"""
    started = pyqtSignal()  # 线程开始
    progress = pyqtSignal(str, int, int)  # 进度更新: 当前文件, 当前索引, 总文件数
    finished = pyqtSignal(list, str)  # 处理完成: 结果列表, 日期字符串
    error = pyqtSignal(str)  # 错误消息
    message = pyqtSignal(str)  # 普通消息


class BagWorker(Thread):
    """后台处理线程，避免界面卡顿"""
    def __init__(self, bag_files, signals):
        super().__init__()
        self.bag_files = bag_files  # 要处理的bag文件列表
        self.signals = signals  # 信号对象
        self._is_running = True  # 运行标志
        
    def run(self):
        """线程主函数"""
        self.signals.started.emit()  # 发送开始信号
        all_results = []  # 存储所有结果
        
        # 从第一个文件名提取日期
        date_str = None
        if self.bag_files:
            first_filename = os.path.basename(self.bag_files[0])
            date_str = extract_date_from_filename(first_filename)
        
        # 如果未提取到日期，使用默认值
        if not date_str:
            date_str = "nodate"
        
        # 遍历处理每个bag文件
        for i, bag_path in enumerate(self.bag_files, 1):
            if not self._is_running:  # 检查停止标志
                break
                
            try:
                # 更新进度
                filename = os.path.basename(bag_path)
                self.signals.progress.emit(filename, i, len(self.bag_files))
                
                # 提取时间戳
                rows = extract_timestamps_from_bag(bag_path)
                all_results.extend(rows)
                
                # 发送处理完成消息
                self.signals.message.emit(f"已处理: {filename}")
                
            except Exception as e:
                # 发送错误消息
                error_msg = f"处理文件 {os.path.basename(bag_path)} 失败: {str(e)}"
                self.signals.error.emit(error_msg)
                continue
        
        # 发送完成信号
        if self._is_running:
            self.signals.finished.emit(all_results, date_str)
            
    def stop(self):
        """停止线程"""
        self._is_running = False


class MainWindow(QMainWindow):
    """主窗口类"""
    def __init__(self):
        super().__init__()
        self.worker = None  # 工作线程
        self.setup_ui()  # 初始化界面
        
    def setup_ui(self):
        """设置用户界面"""
        self.setWindowTitle("角雷达时间戳提取工具")
        self.setGeometry(100, 100, 600, 500)  # 窗口位置和大小
        self.setMinimumSize(500, 400)  # 最小尺寸
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 标题
        title = QLabel("角雷达ADAS报警-时间戳提取工具")
        title.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 文件夹选择区域
        folder_group = QGroupBox("数据源")
        folder_layout = QVBoxLayout()
        
        hbox = QHBoxLayout()
        hbox.addWidget(QLabel("Bag文件夹:"))
        self.bag_dir_edit = QLineEdit()
        self.bag_dir_edit.setPlaceholderText("请选择包含bag文件的文件夹...")
        hbox.addWidget(self.bag_dir_edit)
        self.browse_btn = QPushButton("浏览...")
        self.browse_btn.clicked.connect(self.browse_bag_dir)  # 连接点击事件
        hbox.addWidget(self.browse_btn)
        folder_layout.addLayout(hbox)
        
        self.file_count_label = QLabel("找到 0 个bag文件")
        self.file_count_label.setStyleSheet("color: gray;")
        folder_layout.addWidget(self.file_count_label)
        folder_group.setLayout(folder_layout)
        layout.addWidget(folder_group)
        
        # 进度显示区域
        progress_group = QGroupBox("处理进度")
        progress_layout = QVBoxLayout()
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        progress_layout.addWidget(self.progress_bar)
        
        self.status_label = QLabel("等待开始...")
        progress_layout.addWidget(self.status_label)
        
        self.current_file_label = QLabel("")
        progress_layout.addWidget(self.current_file_label)
        
        progress_group.setLayout(progress_layout)
        layout.addWidget(progress_group)
        
        # 日志输出区域
        log_group = QGroupBox("处理日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)  # 只读
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.addStretch()  # 左侧弹簧
        
        self.start_btn = QPushButton("开始提取")
        self.start_btn.setMinimumSize(120, 40)
        self.start_btn.clicked.connect(self.start_processing)
        button_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setMinimumSize(100, 40)
        self.stop_btn.setEnabled(False)  # 初始禁用
        self.stop_btn.clicked.connect(self.stop_processing)
        button_layout.addWidget(self.stop_btn)
        
        button_layout.addStretch()  # 右侧弹簧
        layout.addLayout(button_layout)
        
        # 初始化信号连接
        self.worker_signals = WorkerSignals()
        self.worker_signals.started.connect(self.on_worker_started)
        self.worker_signals.progress.connect(self.on_worker_progress)
        self.worker_signals.finished.connect(self.on_worker_finished)
        self.worker_signals.error.connect(self.on_worker_error)
        self.worker_signals.message.connect(self.on_worker_message)
        
    def browse_bag_dir(self):
        """浏览选择文件夹"""
        dir_path = QFileDialog.getExistingDirectory(self, "选择Bag文件夹", str(Path.home()))
        if dir_path:
            self.bag_dir_edit.setText(dir_path)  # 设置路径
            self.scan_bag_files()  # 扫描文件
            
    def scan_bag_files(self):
        """扫描文件夹中的bag文件"""
        bag_dir = self.bag_dir_edit.text().strip()
        if not bag_dir or not os.path.exists(bag_dir):
            self.file_count_label.setText("找到 0 个bag文件")
            return
        
        # 递归查找所有.bag文件
        bag_files = []
        for root, dirs, files in os.walk(bag_dir):
            for file in files:
                if file.lower().endswith('.bag'):
                    bag_files.append(os.path.join(root, file))
        
        # 更新UI
        if bag_files:
            self.file_count_label.setText(f"找到 {len(bag_files)} 个bag文件")
            self.start_btn.setEnabled(True)  # 启用开始按钮
        else:
            self.file_count_label.setText("找到 0 个bag文件")
            self.start_btn.setEnabled(False)  # 禁用开始按钮
            
    def start_processing(self):
        """开始处理bag文件"""
        bag_dir = self.bag_dir_edit.text().strip()
        if not bag_dir or not os.path.exists(bag_dir):
            QMessageBox.warning(self, "警告", "请先选择有效的bag文件夹")
            return
        
        # 获取所有bag文件
        bag_files = []
        for root, dirs, files in os.walk(bag_dir):
            for file in files:
                if file.lower().endswith('.bag'):
                    bag_files.append(os.path.join(root, file))
        
        if not bag_files:
            QMessageBox.warning(self, "警告", "没有找到bag文件")
            return
            
        # 创建输出目录
        output_dir = "OUT"
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"无法创建输出目录: {str(e)}")
            return
        
        # 显示开始消息
        self.log_message(f"开始处理 {len(bag_files)} 个bag文件...")
        
        # 创建并启动工作线程
        self.worker = BagWorker(bag_files, self.worker_signals)
        self.worker.start()
        
        # 更新UI状态
        self.start_btn.setEnabled(False)  # 禁用开始按钮
        self.stop_btn.setEnabled(True)  # 启用停止按钮
        self.browse_btn.setEnabled(False)  # 禁用浏览按钮
        self.status_label.setText("处理中...")
        
    def stop_processing(self):
        """停止处理"""
        if self.worker and self.worker.is_alive():
            self.worker.stop()  # 停止线程
            self.worker.join(timeout=1.0)  # 等待线程结束
            self.log_message("处理已停止")
            
    def on_worker_started(self):
        """工作线程开始回调"""
        self.log_message("开始提取时间戳...")
        self.progress_bar.setRange(0, 100)  # 设置进度范围
        self.progress_bar.setValue(0)  # 重置进度
        
    def on_worker_progress(self, current_file, current_index, total_count):
        """工作线程进度更新回调"""
        if total_count > 0:
            # 计算进度百分比
            progress = int(current_index * 100 / total_count)
            self.progress_bar.setValue(progress)  # 更新进度条
            # 显示当前处理文件
            self.current_file_label.setText(f"正在处理: {current_file} ({current_index}/{total_count})")
        
    def on_worker_finished(self, all_results, date_str):
        """工作线程完成回调"""
        self.save_results(all_results, date_str)  # 保存结果
        
        # 更新UI状态
        self.progress_bar.setValue(100)  # 进度到100%
        self.status_label.setText("处理完成")
        self.current_file_label.setText(f"共提取 {len(all_results)} 个报警事件")
        self.start_btn.setEnabled(True)  # 启用开始按钮
        self.stop_btn.setEnabled(False)  # 禁用停止按钮
        self.browse_btn.setEnabled(True)  # 启用浏览按钮
        
        # 显示完成消息
        self.log_message(f"处理完成！共提取 {len(all_results)} 个报警事件")
        
    def on_worker_error(self, error_msg):
        """工作线程错误回调"""
        self.log_message(f"错误: {error_msg}")  # 记录错误
        
    def on_worker_message(self, msg):
        """工作线程消息回调"""
        self.log_message(msg)  # 记录消息
        
    def save_results(self, results, date_str):
        """保存结果到CSV文件"""
        if not results:
            self.log_message("警告: 没有数据可保存")
            return
            
        # 生成带日期的输出文件名
        output_filename = f"timestamps_{date_str}.csv"
        output_path = os.path.join("OUT", output_filename)
        
        try:
            # CSV表头
            fieldnames = [
                "bag", "radar_id", "warning_type", "func_idx", "source", 
                "start_time", "end_time", "duration_sec", "start_frame", "end_frame"
            ]
            
            # 写入CSV文件
            with open(output_path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()  # 写入表头
                
                for row in results:
                    # 确保所有字段都存在
                    for field in fieldnames:
                        if field not in row:
                            row[field] = ""  # 填充空值
                    writer.writerow(row)  # 写入一行
                    
            # 显示保存成功消息
            self.log_message(f"结果已保存到: {output_path}")
            
        except Exception as e:
            # 保存失败处理
            error_msg = f"保存结果失败: {str(e)}"
            self.log_message(error_msg)
            QMessageBox.critical(self, "错误", error_msg)  # 显示错误对话框
            
    def log_message(self, message):
        """添加日志消息"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        log_text = f"[{timestamp}] {message}"
        self.log_text.append(log_text)  # 添加日志
        
        # 自动滚动到底部
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
        
    def closeEvent(self, event):
        """窗口关闭事件处理"""
        if self.worker and self.worker.is_alive():
            # 如果线程正在运行，提示用户确认
            reply = QMessageBox.question(
                self, '确认退出',
                '处理仍在进行中，确定要退出吗？',
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.worker.stop()  # 停止线程
                self.worker.join(timeout=2.0)  # 等待线程结束
                event.accept()  # 接受关闭事件
            else:
                event.ignore()  # 忽略关闭事件
        else:
            event.accept()  # 直接关闭


def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 设置应用样式
    
    window = MainWindow()  # 创建主窗口
    window.show()  # 显示窗口
    
    sys.exit(app.exec_())  # 进入事件循环


if __name__ == '__main__':
    main()  # 程序入口
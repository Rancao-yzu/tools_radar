#!/usr/bin/env python3

"""
雷达Bag文件车速统计分析工具

功能描述：
1. 自动扫描指定文件夹中的所有ROS bag文件
2. 解析指定的ROS话题消息，从二进制数据中提取车速信息
3. 统计车速在10-20 km/h范围内的帧数及占比
4. 生成详细的分析报告并导出为CSV格式
"""
import rosbag
import struct
import csv
import os
import sys
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                           QTextEdit, QFileDialog, QMessageBox, QProgressBar,
                           QSpinBox, QTableWidget, QTableWidgetItem, QHeaderView,
                           QGroupBox, QFormLayout, QSplitter)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont

class BagAnalyzerThread(QThread):
    """后台分析线程"""
    progress_update = pyqtSignal(int, int, str)  # 当前, 总数, 文件名
    result_ready = pyqtSignal(dict)  # 单个结果
    analysis_done = pyqtSignal(list, str)  # 所有结果, CSV路径
    error_occurred = pyqtSignal(str)  # 错误信息

    def __init__(self, bag_files, topic, env_offset):
        super().__init__()
        self.bag_files = bag_files
        self.topic = topic
        self.env_offset = env_offset
        self.results = []

    def run(self):
        try:
            for i, bag_file in enumerate(self.bag_files, 1):
                self.progress_update.emit(i, len(self.bag_files), os.path.basename(bag_file))
                result = self.extract_speed_statistics(bag_file)
                if result:
                    self.result_ready.emit(result)
                    self.results.append(result)
            
            # 生成CSV
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_csv = f"speed_statistics_{timestamp}.csv"
            self.save_to_csv(self.results, output_csv)
            self.analysis_done.emit(self.results, output_csv)
            
        except Exception as e:
            self.error_occurred.emit(str(e))

    def extract_speed_statistics(self, bag_file):
        """提取车速统计信息"""
        speeds_kmh = []
        valid_frames = 0
        total_frames = 0
        
        try:
            bag = rosbag.Bag(bag_file, 'r')
            
            for topic, msg, t in bag.read_messages(topics=[self.topic]):
                total_frames += 1
                raw_bytes = bytes(msg.outputData)
                
                if len(raw_bytes) < self.env_offset + 4:
                    continue
                
                try:
                    speed_mps = struct.unpack_from('<f', raw_bytes, self.env_offset)[0]
                    speed_kmh = speed_mps * 3.6
                    speeds_kmh.append(speed_kmh)
                    valid_frames += 1
                except:
                    continue
            
            bag.close()
            
            if speeds_kmh:
                # 统计车速在10-20 km/h范围内的帧数
                speed_in_range = [s for s in speeds_kmh if 10 <= s <= 20]
                count_in_range = len(speed_in_range)
                
                # 计算统计信息
                speed_min = min(speeds_kmh)
                speed_max = max(speeds_kmh)
                speed_avg = sum(speeds_kmh) / len(speeds_kmh)
                
                if valid_frames > 0:
                    range_percentage = (count_in_range / valid_frames) * 100
                else:
                    range_percentage = 0
                
                return {
                    'bag_name': os.path.basename(bag_file),
                    'bag_path': bag_file,
                    'total_frames': total_frames,
                    'valid_frames': valid_frames,
                    'count_10_20': count_in_range,
                    'percentage': round(range_percentage, 2),
                    'min_speed': round(speed_min, 1),
                    'max_speed': round(speed_max, 1),
                    'avg_speed': round(speed_avg, 1)
                }
            else:
                return {
                    'bag_name': os.path.basename(bag_file),
                    'bag_path': bag_file,
                    'total_frames': total_frames,
                    'valid_frames': valid_frames,
                    'count_10_20': 0,
                    'percentage': 0.0,
                    'min_speed': 0.0,
                    'max_speed': 0.0,
                    'avg_speed': 0.0
                }
                
        except Exception as e:
            return {
                'bag_name': os.path.basename(bag_file),
                'bag_path': bag_file,
                'total_frames': 0,
                'valid_frames': 0,
                'count_10_20': 0,
                'percentage': 0.0,
                'min_speed': 0.0,
                'max_speed': 0.0,
                'avg_speed': 0.0,
                'error': str(e)
            }

    def save_to_csv(self, results, output_csv):
        """保存结果到CSV"""
        if not results:
            return
        
        with open(output_csv, 'w', newline='', encoding='utf-8-sig') as csvfile:
            fieldnames = [
                '序号',
                'bag文件名',
                '总帧数',
                '有效帧数',
                '10-20 km/h帧数',
                '10-20 km/h占比(%)',
                '最小车速(km/h)',
                '最大车速(km/h)',
                '平均车速(km/h)',
                '文件路径'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            total_valid = 0
            total_count_10_20 = 0
            
            for i, result in enumerate(results, 1):
                writer.writerow({
                    '序号': i,
                    'bag文件名': result['bag_name'],
                    '总帧数': result['total_frames'],
                    '有效帧数': result['valid_frames'],
                    '10-20 km/h帧数': result['count_10_20'],
                    '10-20 km/h占比(%)': f"{result['percentage']:.2f}",
                    '最小车速(km/h)': f"{result['min_speed']:.1f}",
                    '最大车速(km/h)': f"{result['max_speed']:.1f}",
                    '平均车速(km/h)': f"{result['avg_speed']:.1f}",
                    '文件路径': result['bag_path']
                })
                
                total_valid += result['valid_frames']
                total_count_10_20 += result['count_10_20']
            
            # 添加汇总行
            total_percentage = (total_count_10_20 / total_valid * 100) if total_valid > 0 else 0
            
            writer.writerow({})
            writer.writerow({
                '序号': '汇总',
                'bag文件名': f'总计 {len(results)} 个文件',
                '总帧数': sum(r['total_frames'] for r in results),
                '有效帧数': total_valid,
                '10-20 km/h帧数': total_count_10_20,
                '10-20 km/h占比(%)': f"{total_percentage:.2f}",
                '最小车速(km/h)': '',
                '最大车速(km/h)': '',
                '平均车速(km/h)': '',
                '文件路径': ''
            })


class BagAnalyzerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bag_files = []
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("雷达Bag文件车速统计分析")
        self.setGeometry(100, 100, 1000, 700)
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 顶部控制面板
        control_group = QGroupBox("分析设置")
        control_layout = QFormLayout()
        
        # 选择文件夹
        folder_layout = QHBoxLayout()
        self.folder_label = QLabel("未选择文件夹")
        self.folder_label.setStyleSheet("border: 1px solid #ccc; padding: 5px;")
        self.folder_label.setMinimumHeight(30)
        self.select_folder_btn = QPushButton("选择文件夹")
        self.select_folder_btn.clicked.connect(self.select_folder)
        folder_layout.addWidget(self.folder_label, 4)
        folder_layout.addWidget(self.select_folder_btn, 1)
        control_layout.addRow("bag文件文件夹:", folder_layout)
        
        # ROS话题
        self.topic_input = QLineEdit("/wf/corner_radar/lgu_data_1")
        control_layout.addRow("ROS话题:", self.topic_input)
        
        # 环境偏移
        offset_layout = QHBoxLayout()
        self.offset_spin = QSpinBox()
        self.offset_spin.setRange(0, 10000)
        self.offset_spin.setValue(584)
        self.offset_spin.setMinimumWidth(100)
        offset_layout.addWidget(self.offset_spin)
        offset_layout.addStretch()
        control_layout.addRow("环境数据偏移(字节):", offset_layout)
        
        # 文件计数
        self.file_count_label = QLabel("0 个文件")
        control_layout.addRow("bag文件数量:", self.file_count_label)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        self.progress_label = QLabel("就绪")
        main_layout.addWidget(self.progress_label)
        main_layout.addWidget(self.progress_bar)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始分析")
        self.start_btn.clicked.connect(self.start_analysis)
        self.start_btn.setEnabled(False)
        self.start_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        
        self.export_btn = QPushButton("导出CSV")
        self.export_btn.clicked.connect(self.export_csv)
        self.export_btn.setEnabled(False)
        
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.export_btn)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
        # 分割面板
        splitter = QSplitter(Qt.Vertical)
        
        # 结果表格
        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(6)
        self.table_widget.setHorizontalHeaderLabels([
            "序号", "文件名", "有效帧数", "10-20帧数", "占比(%)", "平均车速(km/h)"
        ])
        self.table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        splitter.addWidget(self.table_widget)
        
        # 汇总信息
        summary_group = QGroupBox("汇总统计")
        summary_layout = QFormLayout()
        
        self.total_files_label = QLabel("0")
        self.total_valid_label = QLabel("0")
        self.total_range_label = QLabel("0")
        self.total_percent_label = QLabel("0.00%")
        
        summary_layout.addRow("总文件数:", self.total_files_label)
        summary_layout.addRow("总有效帧数:", self.total_valid_label)
        summary_layout.addRow("总10-20 km/h帧数:", self.total_range_label)
        summary_layout.addRow("总占比:", self.total_percent_label)
        
        summary_group.setLayout(summary_layout)
        splitter.addWidget(summary_group)
        splitter.setSizes([500, 150])
        
        main_layout.addWidget(splitter)
        
    def select_folder(self):
        """选择文件夹"""
        folder = QFileDialog.getExistingDirectory(self, "选择包含bag文件的文件夹")
        if folder:
            self.folder_label.setText(folder)
            self.find_bag_files(folder)
            
    def find_bag_files(self, folder):
        """查找bag文件"""
        self.bag_files = []
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.endswith('.bag'):
                    self.bag_files.append(os.path.join(root, file))
        
        self.file_count_label.setText(f"{len(self.bag_files)} 个文件")
        self.start_btn.setEnabled(len(self.bag_files) > 0)
        
        if self.bag_files:
            self.progress_label.setText(f"找到 {len(self.bag_files)} 个bag文件")
        else:
            self.progress_label.setText("未找到bag文件")
            
    def start_analysis(self):
        """开始分析"""
        if not self.bag_files:
            QMessageBox.warning(self, "警告", "没有找到bag文件！")
            return
            
        # 禁用开始按钮
        self.start_btn.setEnabled(False)
        self.start_btn.setText("分析中...")
        self.progress_bar.setValue(0)
        self.progress_bar.setMaximum(len(self.bag_files))
        
        # 清空表格
        self.table_widget.setRowCount(0)
        
        # 创建并启动分析线程
        self.analyzer_thread = BagAnalyzerThread(
            self.bag_files,
            self.topic_input.text(),
            self.offset_spin.value()
        )
        
        self.analyzer_thread.progress_update.connect(self.update_progress)
        self.analyzer_thread.result_ready.connect(self.add_result_row)
        self.analyzer_thread.analysis_done.connect(self.analysis_completed)
        self.analyzer_thread.error_occurred.connect(self.analysis_error)
        
        self.analyzer_thread.start()
        
    def update_progress(self, current, total, filename):
        """更新进度"""
        self.progress_bar.setValue(current)
        self.progress_label.setText(f"正在分析: {filename} ({current}/{total})")
        
    def add_result_row(self, result):
        """添加结果到表格"""
        row = self.table_widget.rowCount()
        self.table_widget.insertRow(row)
        
        self.table_widget.setItem(row, 0, QTableWidgetItem(str(row + 1)))
        self.table_widget.setItem(row, 1, QTableWidgetItem(result['bag_name']))
        self.table_widget.setItem(row, 2, QTableWidgetItem(str(result['valid_frames'])))
        self.table_widget.setItem(row, 3, QTableWidgetItem(str(result['count_10_20'])))
        self.table_widget.setItem(row, 4, QTableWidgetItem(f"{result['percentage']:.2f}%"))
        self.table_widget.setItem(row, 5, QTableWidgetItem(f"{result['avg_speed']:.1f}"))
        
    def analysis_completed(self, results, csv_path):
        """分析完成"""
        self.start_btn.setEnabled(True)
        self.start_btn.setText("开始分析")
        self.export_btn.setEnabled(True)
        self.progress_label.setText(f"分析完成！结果已保存到: {csv_path}")
        
        # 更新汇总信息
        if results:
            total_valid = sum(r['valid_frames'] for r in results)
            total_count_10_20 = sum(r['count_10_20'] for r in results)
            total_percentage = (total_count_10_20 / total_valid * 100) if total_valid > 0 else 0
            
            self.total_files_label.setText(str(len(results)))
            self.total_valid_label.setText(str(total_valid))
            self.total_range_label.setText(str(total_count_10_20))
            self.total_percent_label.setText(f"{total_percentage:.2f}%")
        
        QMessageBox.information(self, "完成", f"分析完成！\nCSV文件已保存到: {csv_path}")
        
    def analysis_error(self, error_msg):
        """分析出错"""
        self.start_btn.setEnabled(True)
        self.start_btn.setText("开始分析")
        self.progress_label.setText("分析出错")
        QMessageBox.critical(self, "错误", f"分析过程中出错:\n{error_msg}")
        
    def export_csv(self):
        """导出CSV"""
        if not hasattr(self, 'analyzer_thread') or not self.analyzer_thread.results:
            QMessageBox.warning(self, "警告", "没有可导出的数据")
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存CSV文件", "", "CSV文件 (*.csv);;所有文件 (*)"
        )
        
        if file_path:
            try:
                self.analyzer_thread.save_to_csv(self.analyzer_thread.results, file_path)
                QMessageBox.information(self, "成功", f"CSV文件已保存到:\n{file_path}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败:\n{str(e)}")


def main():
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle("Fusion")
    
    window = BagAnalyzerGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
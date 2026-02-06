#!/usr/bin/env python3

"""
雷达Bag文件车速统计分析工具

功能描述：
1. 自动扫描指定文件夹中的所有ROS bag文件
2. 解析指定的ROS话题消息，从二进制数据中提取指定偏移量的数据
3. 根据用户选择的数据类型和大小进行解析
4. 统计数据在用户指定范围内的帧数及占比
5. 生成详细的分析报告并导出为CSV格式
"""
import rosbag
import struct
import csv
import os
import sys
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                           QFileDialog, QMessageBox, QProgressBar,
                           QSpinBox, QDoubleSpinBox, QTableWidget, QTableWidgetItem, 
                           QHeaderView, QGroupBox, QFormLayout, QGridLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor

class DataTypeConfig:
    """数据类型配置类"""
    TYPES = {
        'bool': ('?', 1, 'bool'),
        'uint8_t': ('B', 1, 'uint8_t'),
        'uint16_t': ('H', 2, 'uint16_t'),
        'int16_t': ('h', 2, 'int16_t'),
        'uint32_t': ('I', 4, 'uint32_t'),
        'float': ('f', 4, 'float'),
    }
    
    @classmethod
    def get_format_char(cls, data_type):
        """根据数据类型获取格式字符"""
        if data_type in cls.TYPES:
            return cls.TYPES[data_type][0]
        return 'f'  # 默认float
    
    @classmethod
    def get_size(cls, data_type):
        """根据数据类型获取字节数"""
        if data_type in cls.TYPES:
            return cls.TYPES[data_type][1]
        return 4  # 默认4字节
    
    @classmethod
    def get_description(cls, data_type):
        """根据数据类型获取描述"""
        if data_type in cls.TYPES:
            return cls.TYPES[data_type][2]
        return "未知类型"
    
    @classmethod
    def get_all_types(cls):
        """获取所有支持的数据类型"""
        return list(cls.TYPES.keys())


class BagAnalyzerThread(QThread):
    """后台分析线程"""
    progress_update = pyqtSignal(int, int, str)  # 当前, 总数, 文件名
    result_ready = pyqtSignal(dict)  # 单个结果
    analysis_done = pyqtSignal(list, str)  # 所有结果, CSV路径
    error_occurred = pyqtSignal(str)  # 错误信息
    warning_occurred = pyqtSignal(str, str)  # 警告信息: 文件名, 警告信息

    def __init__(self, bag_files, topic, data_type, offset, range_min, range_max):
        super().__init__()
        self.bag_files = bag_files
        self.topic = topic
        self.data_type = data_type
        self.offset = offset
        self.range_min = range_min
        self.range_max = range_max
        self.results = []
        
        # 设置解包格式
        self.set_unpack_format()

    def set_unpack_format(self):
        """根据数据类型设置解包格式"""
        format_char = DataTypeConfig.get_format_char(self.data_type)
        self.unpack_format = f'<{format_char}'  # 固定为小端字节序
        
        # 计算数据类型大小
        self.data_size = DataTypeConfig.get_size(self.data_type)
        
        # 检查偏移量和数据大小
        if self.offset < 0:
            raise ValueError("偏移量不能为负数")
        
        if self.data_size <= 0:
            raise ValueError("无效的数据类型大小")

    def run(self):
        try:
            for i, bag_file in enumerate(self.bag_files, 1):
                self.progress_update.emit(i, len(self.bag_files), os.path.basename(bag_file))
                result = self.extract_data_statistics(bag_file)
                if result:
                    self.result_ready.emit(result)
                    self.results.append(result)
            
            # 生成CSV
            if self.results:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = "./OUT"
                os.makedirs(output_dir, exist_ok=True)
                output_csv = os.path.join(output_dir, f"data_statistics_{timestamp}.csv")
                self.save_to_csv(self.results, output_csv)
                self.analysis_done.emit(self.results, output_csv)
            
        except Exception as e:
            self.error_occurred.emit(str(e))

    def extract_data_statistics(self, bag_file):
        """提取数据统计信息"""
        values = []
        valid_frames = 0
        total_frames = 0
        warnings = []
        
        try:
            bag = rosbag.Bag(bag_file, 'r')
            
            for topic, msg, t in bag.read_messages(topics=[self.topic]):
                total_frames += 1
                
                # 获取消息原始字节数据
                raw_bytes = self.get_message_bytes(msg)
                if raw_bytes is None:
                    continue
                
                # 检查是否有足够的数据
                if len(raw_bytes) < self.offset + self.data_size:
                    warnings.append(f"偏移量 {self.offset} 超出消息长度 {len(raw_bytes)} 字节")
                    continue
                
                try:
                    # 从指定偏移量提取数据
                    data_bytes = raw_bytes[self.offset:self.offset + self.data_size]
                    value = struct.unpack(self.unpack_format, data_bytes)[0]
                    
                    # 对于bool类型，转换为0或1
                    if self.data_type == 'bool':
                        value = 1 if value else 0
                    
                    values.append(value)
                    valid_frames += 1
                except struct.error as e:
                    warnings.append(f"解包错误: {str(e)}")
                    continue
                except Exception as e:
                    warnings.append(f"数据提取错误: {str(e)}")
                    continue
            
            bag.close()
            
            if values:
                # 统计数据在指定范围内的帧数
                values_in_range = [v for v in values if self.range_min <= v <= self.range_max]
                count_in_range = len(values_in_range)
                
                # 计算统计信息
                data_min = min(values)
                data_max = max(values)
                data_avg = sum(values) / len(values)
                
                if valid_frames > 0:
                    range_percentage = (count_in_range / valid_frames) * 100
                else:
                    range_percentage = 0
                
                result = {
                    'bag_name': os.path.basename(bag_file),
                    'bag_path': bag_file,
                    'total_frames': total_frames,
                    'valid_frames': valid_frames,
                    'count_in_range': count_in_range,
                    'percentage': round(range_percentage, 2),
                    'min_value': data_min,
                    'max_value': data_max,
                    'avg_value': data_avg,
                    'data_size': self.data_size,
                    'data_type': self.data_type
                }
                
                # 如果有警告，添加警告信息
                if warnings:
                    result['warnings'] = warnings
                    for warning in warnings:
                        self.warning_occurred.emit(os.path.basename(bag_file), warning)
                
                return result
            else:
                result = {
                    'bag_name': os.path.basename(bag_file),
                    'bag_path': bag_file,
                    'total_frames': total_frames,
                    'valid_frames': valid_frames,
                    'count_in_range': 0,
                    'percentage': 0.0,
                    'min_value': 0.0,
                    'max_value': 0.0,
                    'avg_value': 0.0,
                    'data_size': self.data_size,
                    'data_type': self.data_type
                }
                
                if warnings:
                    result['warnings'] = warnings
                    for warning in warnings:
                        self.warning_occurred.emit(os.path.basename(bag_file), warning)
                
                return result
                
        except Exception as e:
            error_msg = f"文件处理错误: {str(e)}"
            self.warning_occurred.emit(os.path.basename(bag_file), error_msg)
            return {
                'bag_name': os.path.basename(bag_file),
                'bag_path': bag_file,
                'total_frames': 0,
                'valid_frames': 0,
                'count_in_range': 0,
                'percentage': 0.0,
                'min_value': 0.0,
                'max_value': 0.0,
                'avg_value': 0.0,
                'data_size': self.data_size,
                'data_type': self.data_type,
                'error': error_msg
            }

    def get_message_bytes(self, msg):
        """获取消息的字节数据"""
        # 尝试不同的消息字段获取字节数据
        if hasattr(msg, 'outputData'):
            return bytes(msg.outputData)
        elif hasattr(msg, 'data'):
            return bytes(msg.data)
        elif hasattr(msg, 'serialize'):
            try:
                return msg.serialize()
            except:
                pass
        
        # 尝试将消息转换为字节
        try:
            return bytes(msg)
        except:
            pass
        
        return None

    def save_to_csv(self, results, output_csv):
        """保存结果到CSV"""
        if not results:
            return
        
        with open(output_csv, 'w', newline='', encoding='utf-8-sig') as csvfile:
            # 写入配置信息
            csvfile.write("# 分析配置信息\n")
            csvfile.write(f"# 分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            csvfile.write(f"# ROS话题: {self.topic}\n")
            csvfile.write(f"# 数据类型: {self.data_type} ({self.data_size}字节)\n")
            csvfile.write(f"# 偏移量: {self.offset} 字节\n")
            csvfile.write(f"# 分析范围: {self.range_min} ~ {self.range_max}\n")
            csvfile.write("#\n")
            
            fieldnames = [
                '序号',
                'bag文件名',
                '总帧数',
                '有效帧数',
                '选定帧数',
                '选定帧数占比(%)',
                '最小',
                '最大',
                '平均',
                '文件路径'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            total_frames = 0
            total_valid = 0
            total_count_in_range = 0
            
            for i, result in enumerate(results, 1):
                # 根据数据类型格式化数值
                if result['data_type'] in ['bool', 'uint8_t', 'uint16_t', 'uint32_t']:
                    min_val = str(int(result['min_value']))
                    max_val = str(int(result['max_value']))
                    avg_val = f"{result['avg_value']:.1f}"
                else:
                    min_val = f"{result['min_value']:.3f}"
                    max_val = f"{result['max_value']:.3f}"
                    avg_val = f"{result['avg_value']:.3f}"
                
                writer.writerow({
                    '序号': i,
                    'bag文件名': result['bag_name'],
                    '总帧数': result['total_frames'],
                    '有效帧数': result['valid_frames'],
                    '选定帧数': result['count_in_range'],
                    '选定帧数占比(%)': f"{result['percentage']:.2f}",
                    '最小': min_val,
                    '最大': max_val,
                    '平均': avg_val,
                    '文件路径': result['bag_path']
                })
                
                total_frames += result['total_frames']
                total_valid += result['valid_frames']
                total_count_in_range += result['count_in_range']
            
            # 添加汇总行
            writer.writerow({})
            total_percentage = (total_count_in_range / total_valid * 100) if total_valid > 0 else 0
            
            writer.writerow({
                '序号': '汇总',
                'bag文件名': f'总计 {len(results)} 个文件',
                '总帧数': total_frames,
                '有效帧数': total_valid,
                '选定帧数': total_count_in_range,
                '选定帧数占比(%)': f"{total_percentage:.2f}",
                '最小': '',
                '最大': '',
                '平均': '',
                '文件路径': ''
            })


class BagAnalyzerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bag_files = []
        self.current_results = []
        self.warning_messages = []
        self.selected_data_type = 'float'
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("ROS Bag文件数据统计分析工具")
        self.setGeometry(100, 100, 1200, 700)
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        
        # 文件选择
        file_group = QGroupBox("文件选择")
        file_layout = QFormLayout()
        
        folder_layout = QHBoxLayout()
        self.folder_label = QLabel("未选择文件夹")
        self.folder_label.setStyleSheet("""
            QLabel {
                border: 1px solid #ccc; 
                padding: 5px; 
                background-color: white;
                border-radius: 3px;
            }
        """)
        self.folder_label.setMinimumHeight(30)
        self.select_folder_btn = QPushButton("选择文件夹")
        self.select_folder_btn.clicked.connect(self.select_folder)
        self.select_folder_btn.setStyleSheet("padding: 5px; border-radius: 3px;")
        folder_layout.addWidget(self.folder_label, 4)
        folder_layout.addWidget(self.select_folder_btn, 1)
        file_layout.addRow("bag文件文件夹:", folder_layout)
        
        self.file_count_label = QLabel("0 个文件")
        file_layout.addRow("bag文件数量:", self.file_count_label)
        
        file_group.setLayout(file_layout)
        main_layout.addWidget(file_group)
        
        # 数据分析设置
        data_group = QGroupBox("数据分析设置")
        data_layout = QFormLayout()
        
        # ROS话题
        self.topic_input = QLineEdit("/wf/corner_radar/lgu_data_1")
        data_layout.addRow("ROS话题:", self.topic_input)
        
        # 数据类型
        data_type_layout = QVBoxLayout()
        grid_layout = QGridLayout()
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(5)
        
        all_types = DataTypeConfig.get_all_types()
        descriptions = [DataTypeConfig.get_description(t) for t in all_types]
        
        row, col = 0, 0
        for i, (type_name, description) in enumerate(zip(all_types, descriptions)):
            btn = QPushButton(description)
            btn.setCheckable(True)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda checked, t=type_name: self.on_data_type_selected(t))
            btn.setProperty("type", type_name)
            setattr(self, f'{type_name.replace("_", "")}_btn', btn)
            
            grid_layout.addWidget(btn, row, col)
            col += 1

        
        data_type_layout.addLayout(grid_layout)
        data_layout.addRow("数据类型:", data_type_layout)
        
        # 偏移量
        self.offset_spin = QSpinBox()
        self.offset_spin.setRange(0, 10000)
        self.offset_spin.setSingleStep(1)
        self.offset_spin.setMinimumWidth(150)
        self.offset_spin.setValue(584)
        data_layout.addRow("数据偏移(字节):", self.offset_spin)
        
        # 数据范围
        range_layout = QHBoxLayout()
        self.range_min_spin = QDoubleSpinBox()
        self.range_min_spin.setRange(-10000.0, 10000.0)
        self.range_min_spin.setDecimals(3)
        self.range_min_spin.setMinimumWidth(100)
        self.range_min_spin.setValue(2.7777)
        
        self.range_max_spin = QDoubleSpinBox()
        self.range_max_spin.setRange(-10000.0, 10000.0)
        self.range_max_spin.setDecimals(3)
        self.range_max_spin.setMinimumWidth(100)
        self.range_max_spin.setValue(5.5555)
        
        range_layout.addWidget(QLabel("从"))
        range_layout.addWidget(self.range_min_spin)
        range_layout.addWidget(QLabel("到"))
        range_layout.addWidget(self.range_max_spin)
        range_layout.addWidget(QLabel("(包含两端)"))
        range_layout.addStretch()
        data_layout.addRow("分析范围:", range_layout)
        
        data_group.setLayout(data_layout)
        main_layout.addWidget(data_group)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("开始分析")
        self.start_btn.clicked.connect(self.start_analysis)
        self.start_btn.setEnabled(False)
        self.start_btn.setStyleSheet("""
            QPushButton {
                font-weight: bold; 
                padding: 8px; 
                background-color: #4CAF50; 
                color: white;
                border-radius: 4px;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        
        button_layout.addWidget(self.start_btn)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
        # 警告信息显示
        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet("color: orange; padding: 5px;")
        self.warning_label.setVisible(False)
        main_layout.addWidget(self.warning_label)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        self.progress_label = QLabel("就绪")
        main_layout.addWidget(self.progress_label)
        main_layout.addWidget(self.progress_bar)
        
        # 结果表格
        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(9)
        self.table_widget.setHorizontalHeaderLabels([
            "序号", "文件名", "总帧数", "有效帧数", "选定帧数", 
            "占比(%)", "最小值", "最大值", "平均值"
        ])
        header = self.table_widget.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        for i in range(2, 9):
            header.setSectionResizeMode(i, QHeaderView.ResizeToContents)
        
        main_layout.addWidget(self.table_widget)
        
        # 汇总统计（单行显示）
        self.setup_summary_bar()
        main_layout.addWidget(self.summary_bar)
        
        # 设置默认选择
        self.float_btn.setChecked(True)
    
    def setup_summary_bar(self):
        """设置汇总统计栏"""
        self.summary_bar = QGroupBox("汇总统计")
        summary_layout = QHBoxLayout(self.summary_bar)
        summary_layout.setContentsMargins(10, 5, 10, 5)
        summary_layout.setSpacing(20)
        
        # 初始化汇总标签
        self.total_files_label = QLabel("0")
        self.total_frames_label = QLabel("0")
        self.total_valid_label = QLabel("0")
        self.total_range_label = QLabel("0")
        self.total_percent_label = QLabel("0.00%")
        self.data_type_label = QLabel("-")
        self.data_range_label = QLabel("-")
        
        summary_labels = [
            ("总文件数:", self.total_files_label),
            ("总帧数:", self.total_frames_label),
            ("有效帧数:", self.total_valid_label),
            ("选定帧数:", self.total_range_label),
            ("总占比:", self.total_percent_label),
            ("类型:", self.data_type_label),
            ("范围:", self.data_range_label)
        ]
        
        for text, value_label in summary_labels:
            label = QLabel(text)
            label.setStyleSheet("font-weight: bold;")
            value_label.setStyleSheet("""
                QLabel {
                    background-color: #f0f0f0;
                    border: 1px solid #ddd;
                    padding: 2px 6px;
                    border-radius: 3px;
                    min-width: 60px;
                }
            """)
            value_label.setAlignment(Qt.AlignCenter)
            summary_layout.addWidget(label)
            summary_layout.addWidget(value_label)
        
        summary_layout.addStretch()
    
    def on_data_type_selected(self, data_type):
        """数据类型选择"""
        # 取消其他按钮的选择
        for type_name in DataTypeConfig.get_all_types():
            btn = getattr(self, f'{type_name.replace("_", "")}_btn', None)
            if btn and type_name != data_type:
                btn.setChecked(False)
        
        self.selected_data_type = data_type
    
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
        
        # 验证输入
        if not self.topic_input.text().strip():
            QMessageBox.warning(self, "警告", "请输入ROS话题！")
            return
        
        if self.range_min_spin.value() > self.range_max_spin.value():
            QMessageBox.warning(self, "警告", "范围最小值不能大于最大值！")
            return
        
        # 清空警告信息
        self.warning_messages = []
        self.warning_label.clear()
        self.warning_label.setVisible(False)
        
        # 更新界面状态
        self.start_btn.setEnabled(False)
        self.start_btn.setText("分析中...")
        self.progress_bar.setValue(0)
        self.progress_bar.setMaximum(len(self.bag_files))
        
        # 清空表格
        self.table_widget.setRowCount(0)
        self.table_widget.setHorizontalHeaderLabels([
            "序号", "文件名", "总帧数", "有效帧数", "选定帧数", 
            "占比(%)", "最小值", "最大值", "平均值"
        ])
        
        # 创建并启动分析线程
        self.analyzer_thread = BagAnalyzerThread(
            self.bag_files,
            self.topic_input.text().strip(),
            self.selected_data_type,
            self.offset_spin.value(),
            self.range_min_spin.value(),
            self.range_max_spin.value()
        )
        
        self.analyzer_thread.progress_update.connect(self.update_progress)
        self.analyzer_thread.result_ready.connect(self.add_result_row)
        self.analyzer_thread.analysis_done.connect(self.analysis_completed)
        self.analyzer_thread.error_occurred.connect(self.analysis_error)
        self.analyzer_thread.warning_occurred.connect(self.add_warning)
        
        self.analyzer_thread.start()
    
    def update_progress(self, current, total, filename):
        """更新进度"""
        self.progress_bar.setValue(current)
        self.progress_label.setText(f"正在分析: {filename} ({current}/{total})")
    
    def add_result_row(self, result):
        """添加结果到表格"""
        row = self.table_widget.rowCount()
        self.table_widget.insertRow(row)
        
        # 序号
        self.table_widget.setItem(row, 0, QTableWidgetItem(str(row + 1)))
        
        # 文件名
        self.table_widget.setItem(row, 1, QTableWidgetItem(result['bag_name']))
        
        # 总帧数
        self.table_widget.setItem(row, 2, QTableWidgetItem(str(result['total_frames'])))
        
        # 有效帧数
        self.table_widget.setItem(row, 3, QTableWidgetItem(str(result['valid_frames'])))
        
        # 选定帧数
        self.table_widget.setItem(row, 4, QTableWidgetItem(str(result['count_in_range'])))
        
        # 占比
        percentage_item = QTableWidgetItem(f"{result['percentage']:.2f}%")
        if result['percentage'] > 0:
            percentage_item.setBackground(QColor(220, 255, 220))  # 浅绿色背景
        self.table_widget.setItem(row, 5, percentage_item)
        
        # 根据数据类型格式化显示
        data_type = result.get('data_type', 'float')
        
        # 最小值
        min_value = result['min_value']
        if data_type in ['bool', 'uint8_t', 'uint16_t', 'uint32_t']:
            min_text = str(int(min_value))
        elif isinstance(min_value, float):
            min_text = f"{min_value:.3f}"
        else:
            min_text = str(min_value)
        self.table_widget.setItem(row, 6, QTableWidgetItem(min_text))
        
        # 最大值
        max_value = result['max_value']
        if data_type in ['bool', 'uint8_t', 'uint16_t', 'uint32_t']:
            max_text = str(int(max_value))
        elif isinstance(max_value, float):
            max_text = f"{max_value:.3f}"
        else:
            max_text = str(max_value)
        self.table_widget.setItem(row, 7, QTableWidgetItem(max_text))
        
        # 平均值
        avg_value = result['avg_value']
        if data_type in ['bool', 'uint8_t', 'uint16_t', 'uint32_t']:
            avg_text = f"{avg_value:.1f}"
        elif isinstance(avg_value, float):
            avg_text = f"{avg_value:.3f}"
        else:
            avg_text = str(avg_value)
        self.table_widget.setItem(row, 8, QTableWidgetItem(avg_text))
        
        # 保存结果
        self.current_results.append(result)
    
    def add_warning(self, filename, warning_msg):
        """添加警告信息"""
        self.warning_messages.append(f"{filename}: {warning_msg}")
        if len(self.warning_messages) <= 5:  # 只显示最近5条警告
            self.warning_label.setText(f"警告: {warning_msg}")
            self.warning_label.setVisible(True)
    
    def analysis_completed(self, results, csv_path):
        """分析完成"""
        self.start_btn.setEnabled(True)
        self.start_btn.setText("开始分析")
        self.progress_label.setText(f"分析完成！结果已保存到: {csv_path}")
        
        # 显示警告信息总数
        if self.warning_messages:
            self.warning_label.setText(f"有 {len(self.warning_messages)} 条警告信息，详情见控制台输出")
            self.warning_label.setVisible(True)
            
            # 打印所有警告到控制台
            print(f"\n{'='*60}")
            print("分析警告信息：")
            for warning in self.warning_messages:
                print(f"  - {warning}")
            print(f"{'='*60}\n")
        
        # 更新汇总信息
        if results:
            total_frames = sum(r['total_frames'] for r in results)
            total_valid = sum(r['valid_frames'] for r in results)
            total_count_in_range = sum(r['count_in_range'] for r in results)
            total_percentage = (total_count_in_range / total_valid * 100) if total_valid > 0 else 0
            
            data_type = self.selected_data_type
            range_min = self.range_min_spin.value()
            range_max = self.range_max_spin.value()
            
            self.total_files_label.setText(str(len(results)))
            self.total_frames_label.setText(str(total_frames))
            self.total_valid_label.setText(str(total_valid))
            self.total_range_label.setText(str(total_count_in_range))
            self.total_percent_label.setText(f"{total_percentage:.2f}%")
            self.data_type_label.setText(f"{data_type}")
            self.data_range_label.setText(f"{range_min:.1f}~{range_max:.1f}")
        
        QMessageBox.information(self, "完成", f"分析完成！\nCSV文件已保存到: {csv_path}")
    
    def analysis_error(self, error_msg):
        """分析出错"""
        self.start_btn.setEnabled(True)
        self.start_btn.setText("开始分析")
        self.progress_label.setText("分析出错")
        QMessageBox.critical(self, "错误", f"分析过程中出错:\n{error_msg}")
    

def main():
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle("Fusion")
    
    window = BagAnalyzerGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
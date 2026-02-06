#!/usr/bin/env python3

"""
雷达Bag文件目标信息提取工具 -36字节目标结构体

功能描述：
在解析每个数据帧时，程序首先读取8字节的帧头信息，其中包括目标数量（sgu_num）。
由于每个目标信息固定占用36字节，程序通过公式 8 + (sgu_num * 36) 计算出该帧的预期总长度。
然后按顺序解析每个36字节的目标结构体，提取其中信息。
"""
import rosbag
import struct
import csv
import os
import sys
import re
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                           QFileDialog, QMessageBox, QProgressBar,
                           QSpinBox, QGroupBox, QFormLayout, QTextEdit)

class BagAnalyzer:
    def __init__(self, bag_files, frame_header, vehicle_id, custom_field, output_folder):
        self.bag_files = bag_files
        self.frame_header = frame_header
        self.vehicle_id = vehicle_id
        self.custom_field = custom_field
        self.output_folder = output_folder
        
        # 定义4个雷达话题
        self.radar_topics = [
            ("/wf/corner_radar/lgu_data_1", "WFRAFL"),   # 前左雷达
            ("/wf/corner_radar/lgu_data_2", "WFRAFR"),   # 前右雷达
            ("/wf/corner_radar/lgu_data_3", "WFRARL"),   # 后左雷达
            ("/wf/corner_radar/lgu_data_4", "WFRARR"),   # 后右雷达
        ]
        
        # 存储每个雷达的目标数据
        self.radar_targets = {
            "WFRAFL": [],
            "WFRAFR": [],
            "WFRARL": [],
            "WFRARR": [],
        }
        self.csv_files = []  # 存储生成的CSV文件路径
        
        # 添加进度回调函数
        self.progress_callback = None
        self.done_callback = None
        self.error_callback = None

    def set_callbacks(self, progress_callback, done_callback, error_callback):
        """设置回调函数替代信号"""
         # 保存这三个方法，以便后续调用
        self.progress_callback = progress_callback
        self.done_callback = done_callback
        self.error_callback = error_callback

    def run(self):
        try:
            # 创建输出目录
            current_dir = os.getcwd()
            gt_dir = os.path.join(current_dir, "OUT")
            full_output_dir = os.path.join(gt_dir, self.output_folder)
            os.makedirs(full_output_dir, exist_ok=True)
            
            # 初始化进度
            total_files = len(self.bag_files)
            total_radars = len(self.radar_topics)
            total_tasks = total_files * (total_radars + 4)
            
            task_count = 0
            for i, bag_file in enumerate(self.bag_files, 1):
                bag_name = os.path.basename(bag_file)
                
                for topic, radar_name in self.radar_topics:
                    task_count += 1
                    if self.progress_callback:
                        self.progress_callback(task_count, total_tasks, bag_name, f"处理{radar_name}...")
                    
                    # 处理每个雷达话题
                    self.process_radar_topic(bag_file, bag_name, topic, radar_name)
            
                # 为每个雷达生成CSV文件
                for radar_name in self.radar_targets.keys():
                    if self.radar_targets[radar_name]:
                        csv_file = self.save_radar_to_csv(radar_name)
                        if csv_file:
                            self.csv_files.append(csv_file)
                            task_count += 1
                            if self.progress_callback:
                                self.progress_callback(task_count, total_tasks, "", f"{radar_name}CSV完成")

                self.radar_targets = {name: [] for name in self.radar_targets.keys()}
            
            if self.done_callback:
                self.progress_callback(total_tasks, total_tasks, "", f"完成")
                self.done_callback([], self.csv_files)
            
        except Exception as e:
            if self.error_callback:
                self.error_callback(str(e))

    def process_radar_topic(self, bag_file, bag_name, topic, radar_name):
        """处理单个雷达话题"""
        try:
            bag = rosbag.Bag(bag_file, 'r')
            
            for msg_topic, msg, t in bag.read_messages(topics=[topic]):
                if not hasattr(msg, 'outputData'):
                    continue
                    
                raw_bytes = bytes(msg.outputData)
                
                # 检查最小长度：至少要有完整的头部
                if len(raw_bytes) < 8:
                    continue

                offset = 0
                
                # uint16_t headOut
                frame_header_bytes = struct.unpack_from('<H', raw_bytes, offset)[0]
                if frame_header_bytes != self.frame_header:  # 检查帧头
                    continue  # 跳过帧头不匹配的帧
                
                offset += 2
                
                # uint16_t frameID
                frame_id = struct.unpack_from('<H', raw_bytes, offset)[0]
                offset += 2
                
                # uint16_t LGUNum
                lgu_num = struct.unpack_from('<H', raw_bytes, offset)[0]
                offset += 2
                
                # uint8_t SGUNum (目标数量)
                sgu_num = struct.unpack_from('<B', raw_bytes, offset)[0]
                offset += 1
                
                # uint8_t padding
                padding = struct.unpack_from('<B', raw_bytes, offset)[0]
                offset += 1
                
                # 现在 offset = 8 字节，正好是 objTrans 数组的开始，验证数据长度是否足够
                expected_length = 8 + (sgu_num * 36)  # 头部8字节 + 目标数据
                if len(raw_bytes) < expected_length:
                    continue
                
                # 解析每个目标
                target_count = min(sgu_num, 16)  # 最多16个目标
                for obj_idx in range(target_count):
                    if offset + 36 <= len(raw_bytes):
                        target_info = self.parse_single_target(raw_bytes[offset:offset+36])
                        if target_info:
                            # 添加帧信息和bag文件信息
                            target_info.update({
                                'bag_file': bag_name,
                                'radar_name': radar_name,
                                'topic': topic,
                                'frame_id': frame_id,
                                'lgu_num': lgu_num,
                                'sgu_num': sgu_num,
                                'padding': padding,
                                'target_index': obj_idx + 1,
                                'timestamp': t.to_sec(),
                            })
                            self.radar_targets[radar_name].append(target_info)
                        offset += 36  # 每个目标36字节
            
            bag.close()
                
        except Exception as e:
            print(f"处理雷达话题 {topic} 时出错: {e}")
            import traceback
            traceback.print_exc()

    def save_radar_to_csv(self, radar_name):
        """保存雷达数据到CSV，命名"""
        targets = self.radar_targets[radar_name]
        if not targets:
            print(f"{radar_name} 没有找到目标数据")
            return None
        
        # 从第一个bag文件中提取日期和时间
        first_target = targets[0]
        bag_name = first_target['bag_file']
        bag_name_no_ext = os.path.splitext(bag_name)[0]

        # 从bag文件名中提取日期和时间
        date = "unknown_date"
        time = "unknown_time"
        
        patterns = [
            # 格式: *YYYY-MM-DD-HH-MM-SS*
            r'.*(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2}).*',
            # 格式: *YYYYMMDD_HHMMSS*
            r'.*(\d{8})_(\d{6}).*',
            # 格式: *YYYY_MM_DD_HH_MM_SS*
            r'.*(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2}).*',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, bag_name_no_ext)
            if match:
                if len(match.groups()) == 6:
                    # YYYY-MM-DD-HH-MM-SS 格式
                    date = f"{match.group(1)}{match.group(2)}{match.group(3)}"
                    time = f"{match.group(4)}{match.group(5)}{match.group(6)}"
                elif len(match.groups()) == 2:
                    # YYYYMMDD_HHMMSS 格式
                    date = match.group(1)
                    time = match.group(2)
                break
        
        
        # 创建输出目录
        current_dir = os.getcwd()
        gt_dir = os.path.join(current_dir, "OUT")
        full_output_dir = os.path.join(gt_dir, self.output_folder)
        os.makedirs(full_output_dir, exist_ok=True)
        
        # 生成CSV文件名
        csv_filename = f"{self.vehicle_id}_{radar_name}_{date}_{time}_{self.custom_field}.csv"
        output_csv = os.path.join(full_output_dir, csv_filename)
        
        with open(output_csv, 'w', newline='', encoding='utf-8-sig') as csvfile:
            fieldnames = [
                '序号',
                '帧ID',
                '帧时间戳',
                '目标索引',
                '目标ID',
                '目标类型',
                '横向距离X(m)',
                '纵向距离Y(m)',
                '目标长度(m)',
                '目标宽度(m)',
                '航向角(deg)',
                '动态标志',
                '生命周期',
                '相对速度X(m/s)',
                '相对速度Y(m/s)',
                '绝对速度X(m/s)',
                '绝对速度Y(m/s)',
            ]
            
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for i, target in enumerate(targets, 1):
                writer.writerow({
                    '序号': i,
                    '帧ID': target['frame_id'],
                    '帧时间戳': f"{target['timestamp']:.6f}",
                    '目标索引': target['target_index'],
                    '目标ID': target['obj_id'],
                    '目标类型': target['obj_type'],
                    '横向距离X(m)': f"{target['dist_x']:.2f}",
                    '纵向距离Y(m)': f"{target['dist_y']:.2f}",
                    '目标长度(m)': f"{target['length']:.2f}",
                    '目标宽度(m)': f"{target['width']:.2f}",
                    '航向角(deg)': f"{target['yaw_ang']:.2f}",
                    '动态标志': target['dyn_flg'],
                    '生命周期': target['life_cycle'],
                    '相对速度X(m/s)': f"{target['vel_x']:.2f}",
                    '相对速度Y(m/s)': f"{target['vel_y']:.2f}",
                    '绝对速度X(m/s)': f"{target['vel_abs_x']:.2f}",
                    '绝对速度Y(m/s)': f"{target['vel_abs_y']:.2f}",
                })
        
        return output_csv

    def parse_single_target(self, obj_data):
        """解析单个目标结构体 - 36字节版本"""
        try:
            if len(obj_data) < 36:
                return None
                
            offset = 0
            result = {}
            
            # 1-2: int16_t distX (m) - 压缩：实际值 = 值/100
            result['dist_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 3-4: int16_t distY (m) - 压缩：实际值 = 值/100
            result['dist_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 5-6: uint16_t length (m) - 压缩：实际值 = 值/100
            result['length'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 7-8: uint16_t width (m) - 压缩：实际值 = 值/100
            result['width'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 9-10: int16_t yawAng (deg) - 压缩：实际值 = 值/100
            result['yaw_ang'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 11: uint8_t objID
            result['obj_id'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # 12: uint8_t objType
            obj_type_raw = struct.unpack_from('<B', obj_data, offset)[0]
            result['obj_type'] = obj_type_raw
            offset += 1
            
            # 13: uint8_t dynFlg
            result['dyn_flg'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # 14: int8_t objBsdWarningFlag
            result['bsd_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 15: int8_t objLcaWarningFlag
            result['lca_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 16: int8_t objDowWarningFlag
            result['dow_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 17: int8_t objRcwWarningFlag
            result['rcw_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 18: int8_t objRctaWarningFlag
            result['rcta_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 19: int8_t objRctbWarningFlag
            result['rctb_warning'] = struct.unpack_from('<b', obj_data, offset)[0]
            offset += 1
            
            # 20: uint8_t referPt
            result['refer_pt'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # 21-22: uint16_t lifeCycle
            result['life_cycle'] = struct.unpack_from('<H', obj_data, offset)[0]
            offset += 2
            
            # 23-24: int16_t velX (m/s) - 压缩：实际值 = 值/100
            result['vel_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 25-26: int16_t velY (m/s) - 压缩：实际值 = 值/100
            result['vel_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 27-28: int16_t velAbsX (m/s) - 压缩：实际值 = 值/100
            result['vel_abs_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 29-30: int16_t velAbsY (m/s) - 压缩：实际值 = 值/100
            result['vel_abs_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 31-32: uint16_t historyMovDist (m) - 压缩：实际值 = 值/100
            result['history_mov_dist'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 33-34: uint16_t fTTC (s) - 压缩：实际值 = 值/100
            result['f_ttc'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # 35-36: int16_t fDDCI - 压缩：实际值 = 值/100
            result['f_ddci'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            return result
            
        except Exception as e:
            return None


class BagAnalyzerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bag_files = []
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("雷达目标信息提取工具")
        self.setGeometry(100, 100, 660, 600)
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 顶部控制面板
        control_group = QGroupBox("提取设置")
        control_layout = QFormLayout()
        
        # 选择文件夹
        folder_layout = QHBoxLayout()
        self.folder_label = QLabel("未选择文件夹")
        self.folder_label.setStyleSheet("border: 1px solid #ccc; padding: 5px;")
        self.select_folder_btn = QPushButton("选择文件夹")
        self.select_folder_btn.clicked.connect(self.select_folder)
        folder_layout.addWidget(self.folder_label, 4)
        folder_layout.addWidget(self.select_folder_btn, 1)
        control_layout.addRow("bag文件文件夹:", folder_layout)
        
        # 车辆编号
        self.vehicle_input = QLineEdit("F520MR")
        control_layout.addRow("车辆编号:", self.vehicle_input)
        
        # 后缀
        self.custom_input = QLineEdit("GT")
        control_layout.addRow("后缀:", self.custom_input)
        
        # 输出文件夹
        self.output_input = QLineEdit("2026_02_05")
        control_layout.addRow("输出文件夹:", self.output_input)
        
        # 帧头校验
        self.header_input = QSpinBox()
        self.header_input.setRange(0, 65535)
        self.header_input.setValue(0x5AA5)  # 默认帧头
        self.header_input.setMinimumWidth(100)
        self.header_input.setDisplayIntegerBase(16)  # 设置为十六进制显示
        self.header_input.setPrefix("0x")  # 添加前缀，可选
        control_layout.addRow("帧头校验(十六进制):", self.header_input)
        
        # 文件计数
        self.file_count_label = QLabel("0 个文件")
        control_layout.addRow("bag文件数量:", self.file_count_label)
                
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        main_layout.addWidget(self.progress_bar)
        
        
        # 按钮区域
        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始提取")
        self.start_btn.clicked.connect(self.start_analysis)
        self.start_btn.setEnabled(False)
        
        button_layout.addWidget(self.start_btn)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
        # 信息显示
        info_group = QGroupBox("提取信息")
        info_layout = QVBoxLayout()
        
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(290)
        info_layout.addWidget(self.info_text)
        
        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group)
        
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
        self.info_text.append(f"找到 {len(self.bag_files)} 个bag文件")
        
    def start_analysis(self):
        """开始提取"""
        # 获取所有参数
        vehicle_id = self.vehicle_input.text().strip()
        custom_field = self.custom_input.text().strip()
        output_folder = self.output_input.text().strip()
            
        # 禁用开始按钮
        self.start_btn.setEnabled(False)
        self.progress_bar.setValue(0)
        
        self.info_text.clear()
        self.info_text.append(f"开始提取目标信息...")
        self.info_text.append("-" * 50)
        
        # 创建分析对象
        self.analyzer = BagAnalyzer(
            self.bag_files,
            self.header_input.value(),
            vehicle_id,
            custom_field,
            output_folder
        )
        
        # 设置回调函数替代信号
        self.analyzer.set_callbacks(
            self.update_progress,
            self.analysis_completed,
            self.analysis_error
        )
        
        self.analyzer.run()
        
    def update_progress(self, current, total, filename, status):
        """更新进度"""
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)
        
    def analysis_completed(self, results, csv_files):
        """分析完成"""
        self.start_btn.setEnabled(True)
        self.info_text.append(f"分析完成")
        
        
    def analysis_error(self, error_msg):
        """分析出错"""
        self.start_btn.setEnabled(True)
        
        self.info_text.append(f"错误: {error_msg}")
        QMessageBox.critical(self, "错误", f"提取过程中出现错误:\n{error_msg}")


def main():
    app = QApplication(sys.argv)
    app.setStyle("Breeze")    # 设置应用样式
    
    window = BagAnalyzerGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

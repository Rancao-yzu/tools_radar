#!/usr/bin/env python3

"""
雷达Bag文件信息提取工具 - 完整版

功能：
- 目标信息（36字节结构体）
- 自车信息（60字节结构体） 
- 点云信息（16字节结构体）
支持三种数据的选择性提取
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
                           QSpinBox, QGroupBox, QFormLayout, QTextEdit,
                           QCheckBox, QComboBox)

class BagAnalyzer:
    def __init__(self, bag_files, frame_header, vehicle_id, custom_field, 
                 output_folder, extract_options):
        self.bag_files = bag_files
        self.frame_header = frame_header
        self.vehicle_id = vehicle_id
        self.custom_field = custom_field
        self.output_folder = output_folder
        self.extract_options = extract_options  # 新增：提取选项
        
        # 定义4个雷达话题
        self.radar_topics = [
            ("/wf/corner_radar/lgu_data_1", "WFRAFL"),   # 前左雷达
            ("/wf/corner_radar/lgu_data_2", "WFRAFR"),   # 前右雷达
            ("/wf/corner_radar/lgu_data_3", "WFRARL"),   # 后左雷达
            ("/wf/corner_radar/lgu_data_4", "WFRARR"),   # 后右雷达
        ]
        
        # 存储不同类型的数据
        self.radar_targets = {
            "WFRAFL": [],"WFRAFR": [],
            "WFRARL": [],"WFRARR": [],
        }
        
        self.radar_egos = {
            "WFRAFL": [],"WFRAFR": [],
            "WFRARL": [],"WFRARR": [],
        }
        
        self.radar_dots = {
            "WFRAFL": [],"WFRAFR": [],
            "WFRARL": [],"WFRARR": [],
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
            # 计算总任务数
            extract_types = sum(1 for k, v in self.extract_options.items() if v)
            total_files = len(self.bag_files)
            total_radars = len(self.radar_topics)
            total_tasks = total_files * total_radars * extract_types
            
            task_count = 0
            for i, bag_file in enumerate(self.bag_files, 1):
                bag_name = os.path.basename(bag_file)
                
                for topic, radar_name in self.radar_topics:
                    if self.progress_callback:
                        self.progress_callback(task_count, total_tasks, bag_name, f"处理{radar_name}...")
                    
                    # 处理每个雷达话题
                    self.process_radar_topic(bag_file, bag_name, topic, radar_name)
                    task_count += extract_types
            
                # 根据选项生成CSV文件
                if self.extract_options.get('targets', False):
                    for radar_name in self.radar_targets.keys():
                        if self.radar_targets[radar_name]:
                            csv_file = self.save_radar_to_csv(radar_name, 'targets')
                            if csv_file:
                                self.csv_files.append(csv_file)
                                task_count += 1
                                if self.progress_callback:
                                    self.progress_callback(task_count, total_tasks, "", f"{radar_name}目标CSV完成")
                
                if self.extract_options.get('egos', False):
                    for radar_name in self.radar_egos.keys():
                        if self.radar_egos[radar_name]:
                            csv_file = self.save_radar_to_csv(radar_name, 'egos')
                            if csv_file:
                                self.csv_files.append(csv_file)
                                task_count += 1
                                if self.progress_callback:
                                    self.progress_callback(task_count, total_tasks, "", f"{radar_name}自车CSV完成")
                
                if self.extract_options.get('dots', False):
                    for radar_name in self.radar_dots.keys():
                        if self.radar_dots[radar_name]:
                            csv_file = self.save_radar_to_csv(radar_name, 'dots')
                            if csv_file:
                                self.csv_files.append(csv_file)
                                task_count += 1
                                if self.progress_callback:
                                    self.progress_callback(task_count, total_tasks, "", f"{radar_name}点云CSV完成")

                # 清空数据准备下一个bag
                self.radar_targets = {name: [] for name in self.radar_targets.keys()}
                self.radar_egos = {name: [] for name in self.radar_egos.keys()}
                self.radar_dots = {name: [] for name in self.radar_dots.keys()}
            
            if self.done_callback:
                self.progress_callback(total_tasks, total_tasks, "", f"完成")
                self.done_callback([], self.csv_files)
            
        except Exception as e:
            if self.error_callback:
                self.error_callback(str(e))
            import traceback
            traceback.print_exc()

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
                
                # uint8_t isCarSpdOOR
                is_car_spd_oor = struct.unpack_from('<B', raw_bytes, offset)[0]
                offset += 1
                
                # ========== 目标解析 ==========
                if self.extract_options.get('targets', False):
                    expected_length = 8 + (sgu_num * 36)  # 头部8字节 + 目标数据
                    if len(raw_bytes) >= expected_length:
                        target_offset = 8  # 目标从第8字节开始
                        target_count = min(sgu_num, 16) #MAX_size_Sgu = 16
                        for obj_idx in range(target_count):
                            if target_offset + 36 <= len(raw_bytes):
                                target_info = self.parse_single_target(
                                    raw_bytes[target_offset:target_offset+36]
                                )
                                if target_info:
                                    target_info.update({
                                        'bag_file': bag_name,
                                        'radar_name': radar_name,
                                        'topic': topic,
                                        'frame_id': frame_id,
                                        'lgu_num': lgu_num,
                                        'sgu_num': sgu_num,
                                        'target_index': obj_idx + 1,
                                        'timestamp': t.to_sec(),
                                    })
                                    self.radar_targets[radar_name].append(target_info)
                                target_offset += 36
                
                # ========== 中间块解析 (144字节) ==========
                middle_offset = 8 + (16 * 36)  #头部解析+目标解析
                
                # 自车信息 (60字节)
                if self.extract_options.get('egos', False):
                    if middle_offset + 60 <= len(raw_bytes):
                        ego_info = self.parse_ego_info(
                            raw_bytes[middle_offset:middle_offset+60]
                        )
                        if ego_info:
                            ego_info.update({
                                'bag_file': bag_name,
                                'radar_name': radar_name,
                                'topic': topic,
                                'frame_id': frame_id,
                                'lgu_num': lgu_num,
                                'sgu_num': sgu_num,
                                'timestamp': t.to_sec(),
                            })
                            self.radar_egos[radar_name].append(ego_info)
                
                # 校准信息 (12字节)
                calib_offset = middle_offset + 60
                
                # ADAS信息 (12字节)
                adas_offset = calib_offset + 12
                
                # BLD信息 (60字节)
                bld_offset = adas_offset + 12
                
                # ========== 点云解析 ==========
                if self.extract_options.get('dots', False):
                    dots_offset = bld_offset + 60  # 728 字节偏移
                    if len(raw_bytes) >= dots_offset + (lgu_num * 16):
                        for dot_idx in range(lgu_num):
                            dot_offset = dots_offset + (dot_idx * 16)
                            if dot_offset + 16 <= len(raw_bytes):
                                dot_info = self.parse_dot_info(
                                    raw_bytes[dot_offset:dot_offset+16]
                                )
                                if dot_info:
                                    dot_info.update({
                                        'bag_file': bag_name,
                                        'radar_name': radar_name,
                                        'topic': topic,
                                        'frame_id': frame_id,
                                        'lgu_num': lgu_num,
                                        'sgu_num': sgu_num,
                                        'dot_index': dot_idx + 1,
                                        'timestamp': t.to_sec(),
                                    })
                                    self.radar_dots[radar_name].append(dot_info)
            
            bag.close()
                
        except Exception as e:
            print(f"处理雷达话题 {topic} 时出错: {e}")
            import traceback
            traceback.print_exc()

    def parse_single_target(self, obj_data):
        """解析单个目标结构体 (36字节)"""
        try:
            if len(obj_data) < 36:
                return None
                
            offset = 0
            result = {}
            
            # int16_t distX
            result['dist_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t distY
            result['dist_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # uint16_t length
            result['length'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # uint16_t width
            result['width'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t yawAng
            result['yaw_ang'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # uint8_t objID
            result['obj_id'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # uint8_t objType
            result['obj_type'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # uint8_t dynFlg
            result['dyn_flg'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # 跳过8个警告标志 (objBsdWarningFlag ~ objFctbWarningFlag)
            offset += 8
            
            # uint8_t referPt
            result['refer_pt'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # uint8_t lifeCycle
            result['life_cycle'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # uint8_t historyMovDist
            result['history_mov_dist'] = struct.unpack_from('<B', obj_data, offset)[0]
            offset += 1
            
            # int16_t velX
            result['vel_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t velY
            result['vel_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t velAbsX--->accel
            result['accel_x'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t velAbsY--->accel
            result['accel_y'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            offset += 2
            
            # uint16_t fTTC
            result['f_ttc'] = struct.unpack_from('<H', obj_data, offset)[0] / 100.0
            offset += 2
            
            # int16_t fDDCI
            result['f_ddci'] = struct.unpack_from('<h', obj_data, offset)[0] / 100.0
            
            return result
            
        except Exception as e:
            return None

    def parse_ego_info(self, ego_data):
        """解析自车信息 (rbExt_CEnvDataPacket - 60字节)"""
        try:
            if len(ego_data) < 60:
                return None
            
            offset = 0
            result = {}
            
            # float actual_spd
            result['actual_spd'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # float yaw_rate
            result['yaw_rate'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # float lat_accel
            result['lat_accel'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # float long_accel
            result['long_accel'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # unsigned char yaw_rate_sign
            result['yaw_rate_sign'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char actual_gear
            result['actual_gear'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char turn_light_left
            result['turn_light_left'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char turn_light_right
            result['turn_light_right'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # 4个开门标志
            result['open_door_left_top'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['open_door_right_top'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['open_door_left_bottom'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['open_door_right_bottom'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # 4个有效标志
            result['actual_spd_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['yaw_rate_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['lat_accel_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['long_accel_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # float steer_angle
            result['steer_angle'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # 4个轮速
            result['fl_whl_spd'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            result['fr_whl_spd'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            result['rl_whl_spd'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            result['rr_whl_spd'] = struct.unpack_from('<f', ego_data, offset)[0]
            offset += 4
            
            # 4个轮速有效标志
            result['fl_whl_spd_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['fr_whl_spd_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['rl_whl_spd_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            result['rr_whl_spd_valid'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char steer_angle_sign
            result['steer_angle_sign'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char wiper_gear
            result['wiper_gear'] = struct.unpack_from('<B', ego_data, offset)[0]
            offset += 1
            
            # unsigned char padding[2]
            offset += 2
            
            # unsigned int mileage
            result['mileage'] = struct.unpack_from('<I', ego_data, offset)[0]
            
            return result
            
        except Exception as e:
            print(f"解析自车信息错误: {e}")
            return None

    def parse_dot_info(self, dot_data):
        """解析点云信息 (dotOutStrunct - 16字节)"""
        try:
            if len(dot_data) < 16:
                return None
            
            offset = 0
            result = {}
            
            # unsigned short dist
            result['dist'] = struct.unpack_from('<H', dot_data, offset)[0] / 100.0
            offset += 2
            
            # short vel
            result['vel'] = struct.unpack_from('<h', dot_data, offset)[0] / 100.0
            offset += 2
            
            # short angAzi
            result['ang_azi'] = struct.unpack_from('<h', dot_data, offset)[0] / 100.0
            offset += 2
            
            # short angEle
            result['ang_ele'] = struct.unpack_from('<h', dot_data, offset)[0] / 100.0
            offset += 2
            
            # char power
            result['power'] = struct.unpack_from('<b', dot_data, offset)[0]
            offset += 1
            
            # char snr
            result['snr'] = struct.unpack_from('<b', dot_data, offset)[0]
            offset += 1
            
            # char RCS
            result['rcs'] = struct.unpack_from('<b', dot_data, offset)[0]
            offset += 1
            
            # unsigned char idxLocPeer
            result['idx_loc_peer'] = struct.unpack_from('<B', dot_data, offset)[0]
            offset += 1
            
            # unsigned char thetaQly
            result['theta_qly'] = struct.unpack_from('<B', dot_data, offset)[0]
            offset += 1
            
            # unsigned char phiQly
            result['phi_qly'] = struct.unpack_from('<B', dot_data, offset)[0]
            offset += 1
            
            # unsigned char dvQly
            result['dv_qly'] = struct.unpack_from('<B', dot_data, offset)[0]
            offset += 1
            
            # unsigned char is_azi_amb_detected
            result['is_azi_amb_detected'] = struct.unpack_from('<B', dot_data, offset)[0]
            
            return result
            
        except Exception as e:
            return None

    def save_radar_to_csv(self, radar_name, data_type):
        """保存雷达数据到CSV"""
        if data_type == 'targets':
            data_list = self.radar_targets[radar_name]
            type_suffix = "TARGETS"
        elif data_type == 'egos':
            data_list = self.radar_egos[radar_name]
            type_suffix = "EGO"
        elif data_type == 'dots':
            data_list = self.radar_dots[radar_name]
            type_suffix = "DOTS"
        else:
            return None
        
        if not data_list:
            print(f"{radar_name} 没有找到{type_suffix}数据")
            return None
        
        # 从文件名提取日期时间
        first_item = data_list[0]
        bag_name = first_item['bag_file']
        bag_name_no_ext = os.path.splitext(bag_name)[0]

        date = "unknown_date"
        time = "unknown_time"
        
        patterns = [
            r'.*(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2}).*',
            r'.*(\d{8})_(\d{6}).*',
            r'.*(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2}).*',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, bag_name_no_ext)
            if match:
                if len(match.groups()) == 6:
                    date = f"{match.group(1)}{match.group(2)}{match.group(3)}"
                    time = f"{match.group(4)}{match.group(5)}{match.group(6)}"
                elif len(match.groups()) == 2:
                    date = match.group(1)
                    time = match.group(2)
                break
        
        # 创建输出目录
        current_dir = os.getcwd()
        gt_dir = os.path.join(current_dir, "OUTcsv")
        full_output_dir = os.path.join(gt_dir, self.output_folder)
        os.makedirs(full_output_dir, exist_ok=True)
        
        # 生成CSV文件名
        csv_filename = f"{self.vehicle_id}_{radar_name}_{type_suffix}_{date}_{time}_{self.custom_field}.csv"
        output_csv = os.path.join(full_output_dir, csv_filename)
        
        with open(output_csv, 'w', newline='', encoding='utf-8-sig') as csvfile:
            # 根据数据类型选择字段
            if data_type == 'targets':
                fieldnames = [
                    '序号', '帧ID', '帧时间戳', '目标索引',
                    '目标ID', '目标类型', '横向距离X', '纵向距离Y',
                    '目标长度', '目标宽度', '航向角', '动态标志',
                    '生命周期', '相对速度X(m/s)', '相对速度Y(m/s)',
                    'accelX(m/s)', 'accelY(m/s)', 'TTC', 'DDCI'
                ]
            elif data_type == 'egos':
                fieldnames = [
                    '序号', '帧ID', '帧时间戳',
                    '车速(m/s)', '横摆角速度', '横向加速度', '纵向加速度',
                    '横摆角方向', '档位', '左转灯', '右转灯',
                    '左前门', '右前门', '左后门', '右后门',
                    '车速有效', '横摆角有效', '横向加速度有效', '纵向加速度有效',
                    '方向盘角度', '方向盘方向',
                    '左前轮速(km/h)', '右前轮速(km/h)', '左后轮速(km/h)', '右后轮速(km/h)',
                    '左前轮速有效', '右前轮速有效', '左后轮速有效', '右后轮速有效',
                    '雨刮档位', '里程'
                ]
            elif data_type == 'dots':
                fieldnames = [
                    '序号', '帧ID', '帧时间戳', '点索引',
                    '距离', '径向速度(m/s)', '方位角', '俯仰角',
                    '功率(dB)', '信噪比', 'RCS', '索引位置',
                    '方位角质量', '俯仰角质量', '速度解模糊质量', '方位角模糊标志'
                ]
            
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for i, item in enumerate(data_list, 1):
                if data_type == 'targets':
                    writer.writerow({
                        '序号': i,
                        '帧ID': item['frame_id'],
                        '帧时间戳': f"{item['timestamp']:.6f}",
                        '目标索引': item['target_index'],
                        '目标ID': item['obj_id'],
                        '目标类型': item['obj_type'],
                        '横向距离X': f"{item['dist_x']:.2f}",
                        '纵向距离Y': f"{item['dist_y']:.2f}",
                        '目标长度': f"{item['length']:.2f}",
                        '目标宽度': f"{item['width']:.2f}",
                        '航向角': f"{item['yaw_ang']:.2f}",
                        '动态标志': item['dyn_flg'],
                        '生命周期': item['life_cycle'],
                        '相对速度X(m/s)': f"{item['vel_x']:.2f}",
                        '相对速度Y(m/s)': f"{item['vel_y']:.2f}",
                        'accelX(m/s)': f"{item['accel_x']:.2f}",
                        'accelY(m/s)': f"{item['accel_y']:.2f}",
                        'TTC': f"{item['f_ttc']:.2f}",
                        'DDCI': f"{item['f_ddci']:.2f}",
                    })
                elif data_type == 'egos':
                    writer.writerow({
                        '序号': i,
                        '帧ID': item['frame_id'],
                        '帧时间戳': f"{item['timestamp']:.6f}",
                        '车速(m/s)': f"{item['actual_spd']:.2f}",
                        '横摆角速度': f"{item['yaw_rate']:.2f}",
                        '横向加速度': f"{item['lat_accel']:.2f}",
                        '纵向加速度': f"{item['long_accel']:.2f}",
                        '横摆角方向': item['yaw_rate_sign'],
                        '档位': item['actual_gear'],
                        '左转灯': item['turn_light_left'],
                        '右转灯': item['turn_light_right'],
                        '左前门': item['open_door_left_top'],
                        '右前门': item['open_door_right_top'],
                        '左后门': item['open_door_left_bottom'],
                        '右后门': item['open_door_right_bottom'],
                        '车速有效': item['actual_spd_valid'],
                        '横摆角有效': item['yaw_rate_valid'],
                        '横向加速度有效': item['lat_accel_valid'],
                        '纵向加速度有效': item['long_accel_valid'],
                        '方向盘角度': f"{item['steer_angle']:.2f}",
                        '方向盘方向': item['steer_angle_sign'],
                        '左前轮速(km/h)': f"{item['fl_whl_spd']:.2f}",
                        '右前轮速(km/h)': f"{item['fr_whl_spd']:.2f}",
                        '左后轮速(km/h)': f"{item['rl_whl_spd']:.2f}",
                        '右后轮速(km/h)': f"{item['rr_whl_spd']:.2f}",
                        '左前轮速有效': item['fl_whl_spd_valid'],
                        '右前轮速有效': item['fr_whl_spd_valid'],
                        '左后轮速有效': item['rl_whl_spd_valid'],
                        '右后轮速有效': item['rr_whl_spd_valid'],
                        '雨刮档位': item['wiper_gear'],
                        '里程': item['mileage'],
                    })
                elif data_type == 'dots':
                    writer.writerow({
                        '序号': i,
                        '帧ID': item['frame_id'],
                        '帧时间戳': f"{item['timestamp']:.6f}",
                        '点索引': item['dot_index'],
                        '距离': f"{item['dist']:.2f}",
                        '径向速度(m/s)': f"{item['vel']:.2f}",
                        '方位角': f"{item['ang_azi']:.2f}",
                        '俯仰角': f"{item['ang_ele']:.2f}",
                        '功率(dB)': item['power'],
                        '信噪比': item['snr'],
                        'RCS': item['rcs'],
                        '索引位置': item['idx_loc_peer'],
                        '方位角质量': item['theta_qly'],
                        '俯仰角质量': item['phi_qly'],
                        '速度解模糊质量': item['dv_qly'],
                        '方位角模糊标志': item['is_azi_amb_detected'],
                    })
        
        return output_csv


class BagAnalyzerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bag_files = []
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("雷达数据提取工具")
        self.setGeometry(100, 100, 800, 700)
        
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
        header_layout = QHBoxLayout()
        self.header_input = QSpinBox()
        self.header_input.setRange(0, 65535)
        self.header_input.setValue(0x5AA5)
        self.header_input.setMinimumWidth(100)
        self.header_input.setDisplayIntegerBase(16)
        self.header_input.setPrefix("0x")
        header_layout.addWidget(self.header_input)
        control_layout.addRow("帧头校验(十六进制):", header_layout)
        
        # 文件计数
        self.file_count_label = QLabel("0 个文件")
        control_layout.addRow("bag文件数量:", self.file_count_label)
                
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # ========== 提取选项 ==========
        options_group = QGroupBox("提取数据类型 (可多选)")
        options_layout = QHBoxLayout()
        
        self.targets_check = QCheckBox("目标物信息 (Targets)")
        self.targets_check.setChecked(True)
        
        self.egos_check = QCheckBox("自车信息 (Ego Vehicle)")
        self.egos_check.setChecked(False)
        
        self.dots_check = QCheckBox("点云信息 (Point Cloud)")
        self.dots_check.setChecked(False)
        
        options_layout.addWidget(self.targets_check)
        options_layout.addWidget(self.egos_check)
        options_layout.addWidget(self.dots_check)
        

        options_group.setLayout(options_layout)
        main_layout.addWidget(options_group)
        
        # ========== 进度条 ==========
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        main_layout.addWidget(self.progress_bar)
        
        
        # 按钮区域
        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始提取")
        self.start_btn.clicked.connect(self.start_analysis)
        self.start_btn.setEnabled(False)
        self.start_btn.setMinimumHeight(40)
        self.start_btn.setStyleSheet("font-weight: bold;")
        
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
        # 检查至少选择了一种数据类型
        extract_options = {
            'targets': self.targets_check.isChecked(),
            'egos': self.egos_check.isChecked(),
            'dots': self.dots_check.isChecked()
        }
        
        if not any(extract_options.values()):
            QMessageBox.warning(self, "警告", "请至少选择一种要提取的数据类型！")
            return
        
        # 获取参数
        vehicle_id = self.vehicle_input.text().strip()
        custom_field = self.custom_input.text().strip()
        output_folder = self.output_input.text().strip()
            
        # 禁用开始按钮
        self.start_btn.setEnabled(False)
        self.progress_bar.setValue(0)
        
        self.info_text.clear()
        self.info_text.append("=" * 60)
        self.info_text.append("开始提取数据...")
        self.info_text.append("-" * 60)
        
        # 显示选择的提取类型
        selected = []
        if extract_options['targets']: selected.append("目标物")
        if extract_options['egos']: selected.append("自车信息")
        if extract_options['dots']: selected.append("点云")
        self.info_text.append(f"提取类型: {', '.join(selected)}")
        self.info_text.append("-" * 60)
        
        # 创建分析对象
        self.analyzer = BagAnalyzer(
            self.bag_files,
            self.header_input.value(),
            vehicle_id,
            custom_field,
            output_folder,
            extract_options
        )
        
        # 设置回调函数
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

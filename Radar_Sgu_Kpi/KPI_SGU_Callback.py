#!/usr/bin/env python3
"""
雷达-IMU 时间戳匹配工具
简化输出结构
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import rosbag
import datetime
import csv
import os
import numpy as np
import threading
import queue


class TimeStampMatcher:
    def __init__(self, bag_files,  log_queue):
        self.bag_files = bag_files
        self.log_queue = log_queue
        
        # 定义4个雷达话题
        self.radar_topics = [
            ("/wf/objectlist_1", "FL"),   # 前左雷达
            ("/wf/objectlist_2", "FR"),   # 前右雷达
            ("/wf/objectlist_3", "RL"),   # 后左雷达
            ("/wf/objectlist_4", "RR"),   # 后右雷达
        ]
        
        # IMU话题
        self.imu_topic = "/wf/imu_data/parsed"
        
        # 存储IMU数据
        self.imu_data = []     # IMU数据

    def log(self, message):
        """添加日志消息"""
        self.log_queue.put(message)

    def run(self):
        """主处理流程"""
        try:
            total_files = len(self.bag_files)
            self.log(f"开始处理 {total_files} 个bag文件")
            
            for i, bag_file in enumerate(self.bag_files, 1):
                    
                bag_name = os.path.basename(bag_file)
                self.log(f"[{i}/{total_files}] 处理: {bag_name}")

                self.imu_data = []
                
                self.extract_imu_data(bag_file, bag_name)
                
                
                bag_output_dir = self.create_bag_output_dir(bag_name)
                
                output_files = []
                for topic, radar_name in self.radar_topics:
                    
                    self.log(f" 处理雷达: {radar_name}")
                    csv_file = self.process_radar_topic(bag_file, bag_name, topic, radar_name, bag_output_dir)
                    if csv_file:
                        output_files.append(csv_file)
                
                if output_files:
                    self.log(f" 生成 {len(output_files)} 个CSV文件")
                
            self.log("所有文件处理完成！")
                
        except Exception as e:
            self.log(f"处理过程中出现错误: {str(e)}")
            import traceback
            traceback.print_exc()

    def create_bag_output_dir(self, bag_name):
        """为每个bag文件创建独立的输出文件夹"""
        current_dir = os.getcwd()
        base_name = os.path.splitext(bag_name)[0]
        
        main_output_dir = os.path.join(current_dir, "Matched_Results_Recall")
        os.makedirs(main_output_dir, exist_ok=True)
        bag_output_dir = os.path.join(main_output_dir, base_name)
        
        os.makedirs(bag_output_dir, exist_ok=True)
        self.log(f" 创建输出文件夹: {bag_output_dir}")
        return bag_output_dir

    def extract_imu_data(self, bag_file, bag_name):
        """提取IMU数据 - 使用英文字段名"""
        try:
            bag = rosbag.Bag(bag_file, 'r')
            imu_count = 0
            
            for topic, msg, t in bag.read_messages(topics=[self.imu_topic]):
                    
                imu_record = {
                    'bag_file': bag_name,
                    'timestamp': t.to_sec(),
                }

                if t.to_sec() > 1780282077:  # 2026年6月1日的时间戳 1780282077
                    bag.close()
                    raise Exception(f"提取IMU数据时ERROR - TIME EXPIRED -{t.to_sec()} > 2026年6月")
                
                # 字段名映射
                imu_fields = {
                    'Target1CoorX': 'GT_LngDist',        # 纵向距离
                    'Target1CoorY': 'GT_LatDist',     # 横向距离

                    'Target1LngSpeedKPH': 'GT_LngSpeed',    # 纵向速度
                    'Target1LatSpeedms': 'GT_LatSpeed',     # 横向速度

                    'Target1AcceX': 'AcceX_GT',             # 纵向加速度
                    'Target1AcceY': 'AcceY_GT',             # 横向加速度
                    
                    'Target1HeadingDiff': 'YawAng_GT',      # 航向角
                }
                
                # 提取IMU字段
                for msg_field, csv_field in imu_fields.items():
                    value = self.get_imu_message_field(msg, msg_field)
                    
                    # 特殊处理速度单位转换
                    if msg_field in ['Target1LngSpeedKPH']:
                        if value is not None:
                            value = float(value) / 3.6  # km/h -> m/s


                    if msg_field in ['Target1HeadingDiff']:
                        if value is not None:
                            value = float(value)  * (3.1416 / 180.0)  #deg->rad

                    
                    imu_record[csv_field] = float(value) if value is not None else 0.0
                
                self.imu_data.append(imu_record)
                imu_count += 1
            
            bag.close()
            self.log(f" 提取IMU数据: {imu_count} 条")
            
            # 检查是否提取到数据
            if imu_count == 0:
                self.log(" 警告: 没有提取到IMU数据")
            
        except Exception as e:
            self.log(f"提取IMU数据时出错: {e}")
            import traceback
            traceback.print_exc()
            raise

    def get_imu_message_field(self, msg, field_name):
        """获取IMU消息字段值"""
        try:
            # 直接获取属性
            if hasattr(msg, field_name):
                return getattr(msg, field_name)
            
            
            # 尝试从header中获取
            if hasattr(msg, 'header'):
                header = msg.header
                if hasattr(header, field_name):
                    return getattr(header, field_name)

            return None
            
        except Exception as e:
            return None

    def process_radar_topic(self, bag_file, bag_name, topic, radar_name, output_dir):
        """处理单个雷达话题 - 使用ROS消息结构"""
        try:
            bag = rosbag.Bag(bag_file, 'r')
            radar_targets = []
            frame_count = 0
            
            # 处理该雷达的所有消息
            for msg_topic, msg, t in bag.read_messages(topics=[topic]):

                if hasattr(msg, 'header'):
                    header = msg.header  #实际录制时候的时间戳
                
                # 直接使用msg.ObjectsBuffer
                if not hasattr(msg, 'ObjectsBuffer'):
                    continue
                        
                frame_count += 1
                timestamp = t.to_sec() #回灌后的时间戳
                
                # 遍历目标列表
                for obj_idx, obj in enumerate(msg.ObjectsBuffer):
                    
                    # 提取目标信息
                    target_info = {
                        'bag_file': bag_name,
                        'radar_name': radar_name,
                        'topic': topic,
                        'frame_id': frame_count,
                        'target_index': obj_idx + 1,
                        'timestamp': timestamp,
                        'sgu_num': len(msg.ObjectsBuffer),  # 目标数量
                        
                        # 雷达目标数据
                        'RxReal': obj.RxReal if hasattr(obj, 'RxReal') else 0.0,
                        'RyReal': obj.RyReal if hasattr(obj, 'RyReal') else 0.0,
                        'velAbsX': obj.velAbsX if hasattr(obj, 'velAbsX') else 0.0,
                        'velAbsY': obj.velAbsY if hasattr(obj, 'velAbsY') else 0.0,
                        'accelAbsX': obj.accelAbsX if hasattr(obj, 'accelAbsX') else 0.0,
                        'accelAbsY': obj.accelAbsY if hasattr(obj, 'accelAbsY') else 0.0,
                        'yaw_ang': (obj.yawAng if hasattr(obj, 'yawAng') else 0.0)* (3.1416 / 180.0) ,
                        'length': obj.length if hasattr(obj, 'length') else 0.0,
                        'width': obj.width if hasattr(obj, 'width') else 0.0,
                        'existProb': round(obj.existProb * 100) if hasattr(obj, 'existProb') else 0.0,
                        'obj_id': obj.objID if hasattr(obj, 'objID') else 0,
                        'obj_type': obj.objType if hasattr(obj, 'objType') else 0,
                        'dyn_flg': obj.dynFlg if hasattr(obj, 'dynFlg') else 0,
                        'obstProbability': obj.obstProbability if hasattr(obj, 'obstProbability') else 0,
                    }
                    
                    radar_targets.append(target_info)
            
            bag.close()
            
            if radar_targets:
                self.log(f"    处理 {frame_count} 帧, 提取 {len(radar_targets)} 个目标")
                # 时间戳匹配并保存CSV
                csv_file = self.match_and_save_radar_data(bag_name, radar_name, radar_targets, output_dir)
                return csv_file
            else:
                self.log(f"    没有提取到目标数据")
                return None
            
        except Exception as e:
            self.log(f"处理雷达话题 {topic} 时出错: {e}")
            import traceback
            traceback.print_exc()
            return None

    def match_and_save_radar_data(self, bag_name, radar_name, radar_targets, output_dir):
        """时间戳匹配并保存雷达数据"""
        if not radar_targets or not self.imu_data:
            if not radar_targets:
                self.log(f"    没有雷达目标数据")
            if not self.imu_data:
                self.log(f"    没有IMU数据")
            return None
        
        # 创建IMU时间戳数组用于快速查找
        imu_timestamps = np.array([imu['timestamp'] for imu in self.imu_data])
        matched_records = []
        match_count = 0
        skipped_count = 0
        
        for target in radar_targets:
                
            # 找到最近的时间戳
            idx = np.argmin(np.abs(imu_timestamps - target['timestamp']))
            nearest_imu = self.imu_data[idx]
            time_diff = abs(nearest_imu['timestamp'] - target['timestamp'])
            
            # 检查时间差是否在合理范围内
            if time_diff < 0.1:  # 100ms阈值
                matched_record = {
                    'timestamp': target['timestamp'],
                    'frame_id': target['frame_id'],
                    'ID': target['obj_id'],
                    'RxReal': target['RxReal'],
                    'RyReal': target['RyReal'],
                    'Vx': target['velAbsX'],
                    'Vy': target['velAbsY'],
                    'AcceX': target['accelAbsX'],
                    'AcceY': target['accelAbsY'],
                    'YawAng': target['yaw_ang'],
                    'objType': target['obj_type'],
                    'dynFlg': target['dyn_flg'],
                    'length': target['length'],
                    'width': target['width'],
                    'RCS': 0,
                    'existProb': target['existProb'],
                    'obstProbability': target['obstProbability'],
                    
                    # IMU数据
                    'GT_LngDist': nearest_imu.get('GT_LngDist', 0),
                    'GT_LatDist': nearest_imu.get('GT_LatDist', 0),
                    'GT_LngSpeed': nearest_imu.get('GT_LngSpeed', 0),
                    'GT_LatSpeed': nearest_imu.get('GT_LatSpeed', 0),
                    'AcceX_GT': nearest_imu.get('AcceX_GT', 0),
                    'AcceY_GT': nearest_imu.get('AcceY_GT', 0),
                    'YawAng_GT': nearest_imu.get('YawAng_GT', 0),
                    
                    # 元数据
                    'time_diff': time_diff,
                }
                
                matched_records.append(matched_record)
                match_count += 1
            else:
                skipped_count += 1
        
        if match_count > 0:
            self.log(f"    匹配成功: {match_count} 个目标 (跳过 {skipped_count} 个时间差过大的目标)")
            # 保存CSV文件
            return self.save_radar_csv(bag_name, radar_name, matched_records, output_dir)
        else:
            self.log(f"    没有匹配到目标 (跳过 {skipped_count} 个时间差过大的目标)")
            return None

    def save_radar_csv(self, bag_name, radar_name, matched_records, output_dir):
        """保存单个雷达的CSV文件"""
        if not matched_records:
            return None
        
        # 生成CSV文件名
        csv_filename = f"{bag_name}_{radar_name}.csv"
        csv_path = os.path.join(output_dir, csv_filename)
        
        # 字段顺序
        fieldnames = [
            'timestamp', 'frame_id', 'ID', 
            'RxReal', 'RyReal', 
            'Vx', 'Vy', 
            'AcceX', 'AcceY', 
            'YawAng', 
            'objType', 'dynFlg', 
            'length', 'width',
            'RCS', 
            'existProb', 'obstProbability',
            'GT_LngDist', 'GT_LatDist', 
            'GT_LngSpeed', 'GT_LatSpeed',
            'AcceX_GT', 'AcceY_GT', 
            'YawAng_GT',
        ]
        
        with open(csv_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for record in matched_records:
                row = {}
                for field in fieldnames:
                    if field in record:
                        value = record[field]
                        if isinstance(value, float):
                            row[field] = f"{value:.4f}"
                        else:
                            row[field] = str(value)
                    else:
                        row[field] = ""
                writer.writerow(row)
        
        self.log(f"    保存: {csv_filename} ({len(matched_records)} 行)")
        return csv_path


class MatcherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("雷达-IMU 时间戳匹配工具")
        self.root.geometry("700x550")
        
        # 初始化变量
        self.bag_files = []
        self.matcher = None
        self.processing = False
        self.log_queue = queue.Queue()

        
        # 设置UI
        self.setup_ui()

        # 根据初始状态设置按钮
        self.start_button.config(state=tk.DISABLED)
        
        # 启动日志更新线程
        self.update_log()


    def setup_ui(self):
        """设置用户界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # 行计数器
        row = 0
        
        # Bag文件文件夹选择
        ttk.Label(main_frame, text="选择bag文件文件夹:").grid(row=row, column=0, sticky=tk.W, pady=5)
        self.bag_folder_var = tk.StringVar()
        folder_frame = ttk.Frame(main_frame)
        folder_frame.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5)
        self.bag_folder_entry = ttk.Entry(folder_frame, textvariable=self.bag_folder_var, width=50)
        self.bag_folder_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(folder_frame, text="浏览...", command=self.select_folder).pack(side=tk.LEFT, padx=5)
        row += 1
        
        # 雷达信息标签
        self.radar_info_label = ttk.Label(main_frame, text="将处理4个雷达: WFRAFL, WFRAFR, WFRARL, WFRARR")
        self.radar_info_label.grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=5)
        row += 1
        
        # 输出结构说明
        self.struct_info_label = ttk.Label(main_frame, text="每个bag文件将创建一个独立文件夹，包含4个雷达的CSV文件")
        self.struct_info_label.grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=5)
        row += 1
        
        # 开始按钮
        self.start_button = ttk.Button(main_frame, text="开始时间戳匹配", command=self.start_matching)
        self.start_button.grid(row=row, column=0, columnspan=2, pady=10)
        row += 1
        
        # 日志区域标题
        ttk.Label(main_frame, text="处理日志:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(5, 0))
        row += 1
        
        # 日志文本框
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # 配置网格权重
        main_frame.rowconfigure(row, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, wrap=tk.WORD, font=('Consolas', 9))
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

    def select_folder(self):
        """选择文件夹"""
        folder = filedialog.askdirectory(title="选择包含bag文件的文件夹")
        if folder:
            self.bag_folder_var.set(folder)
            self.find_bag_files(folder)
    
    def find_bag_files(self, folder):
        """查找bag文件"""
        self.bag_files = []
        
        for root, dirs, files in os.walk(folder):
            for file in sorted(files):  # 按文件名排序
                if file.endswith('.bag'):
                    bag_path = os.path.join(root, file)
                    self.bag_files.append(bag_path)
        
        file_count = len(self.bag_files)
        
        if file_count > 0:
            self.log(f"找到 {file_count} 个bag文件:")
            for i, file in enumerate(self.bag_files[:5], 1):  # 只显示前5个
                self.log(f"  {i}. {os.path.basename(file)}")
            if file_count > 5:
                self.log(f"  ... 还有 {file_count-5} 个文件")
        else:
            self.log("没有找到bag文件")
                
        self.start_button.config(state=tk.NORMAL if (self.bag_files) else tk.DISABLED)
    
    def log(self, message):
        """添加日志消息"""
        self.log_queue.put(message)
    
    def update_log(self):
        """更新日志显示"""
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.log_text.insert(tk.END, message + "\n")
                self.log_text.see(tk.END)
                self.log_text.update_idletasks()
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.update_log)
    
    def start_matching(self):
        """开始时间戳匹配"""
        if not self.bag_files:
            messagebox.showwarning("警告", "没有找到bag文件！")
            return

        # 禁用开始按钮
        self.start_button.config(state=tk.DISABLED)
        
        # 清空日志
        self.log_text.delete(1.0, tk.END)
        self.log("开始时间戳匹配...")
        self.log("=" * 50)
        
        # 在新线程中处理
        self.matcher = TimeStampMatcher(
            self.bag_files,
            self.log_queue
        )
        
        self.processing = True
        thread = threading.Thread(target=self.run_matcher)
        thread.daemon = True
        thread.start()
        
        # 监控处理线程
        self.monitor_processing()
    
    def run_matcher(self):
        """运行匹配器"""
        self.matcher.run()
        self.processing = False
    
    def monitor_processing(self):
        """监控处理线程状态"""
        if self.processing:
            self.root.after(500, self.monitor_processing)
        else:
            self.start_button.config(state=tk.NORMAL)
            self.log("=" * 50)
            self.log("处理完成！")


def main():
    """主函数"""
    root = tk.Tk()
    root.title("雷达-IMU 时间戳匹配工具")
    root.geometry("700x550")
    
    app = MatcherGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
IMU数据转CSV工具 - Python版本

功能：
1. 扫描指定文件夹中的所有.bag文件
2. 处理/wf/imu_data/parsed话题
3. 使用通用方式读取消息字段
4. 保存为CSV格式，只保存非零字段
"""

import os
import re
import csv
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import threading
import queue


# ROS相关导入
import rosbag


class ImuToCSVApp:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU数据转CSV工具 - Python版")
        self.root.geometry("700x600")
        
        # 处理状态
        self.processing = False
        self.stop_requested = False
        self.log_queue = queue.Queue()
        
        # IMU消息字段映射表
 
        self.field_mapping = {
            'timestamp': 'timestamp',                     # 时间戳（Unix时间，精确到微秒）
            
            # 目标车辆信息
            'GT_Dis': 'Target1Distance',             # 目标车与本车的距离（米）
            'GT_DisX': 'Target1LngDistance',         #纵向距离
            'GT_DisY': 'Target1LatDistance',         # 横向距离

            'GT_HeadingDiff': 'Target1HeadingDiff',        # 目标车与本车的航向角差（度）
            'GT_Angle': 'Target1Angle',                    # 目标车相对本车的角度（度）
            'GT_PitchAngle': 'Target1PitchAngle',          # 目标车俯仰角（度）
            'GT_RollAngle': 'Target1RollAngle',            # 目标车横滚角（度）

            'GT_X': 'Target1CoorX',                       # 目标车在坐标系中的X坐标
            'GT_Y': 'Target1CoorY',                       # 目标车在坐标系中的Y坐标

            'GT_Rel_V': 'Target1RelativeSpeed',            # 目标车相对于本车的速度（米/秒）
            'GT_V': 'Target1SpeedKPH',                     # 目标车绝对速度（千米/小时）,已转换为米/秒
            'GT_Vx': 'Target1LngSpeedKPH',                 # 纵向速度，,已转换为米/秒
            'GT_Vy': 'Target1LatSpeedms',                  # 横向速度，已转换为米/秒
            'GT_AcceX': 'Target1AcceX',                    # 目标车对地纵向加速度（米/秒²）
            'GT_AcceY': 'Target1AcceY',                    # 目标车对地横向加速度（米/秒²）
            'GT_RelAcceX': 'Target1RelLngAcc',
            'GT_RelAcceY': 'Target1RelLatAcc',

            # 自车信息
            'E_V': 'Ego1SpeedKPH',                        # 自车速度（千米/小时）,已转换为米/秒
            'E_YawRate': 'Ego1YawRate',                   # 自车横摆角速度（度/秒）
            'E_AcceX': 'Ego1LngAcce',                     # 自车纵向加速度（米/秒²）
            'E_AcceY': 'Ego1LatAcce',                     # 自车横向加速度（米/秒²）
            'E_Heading': 'Ego1Heading'                     # 自车航向角（度）
        }
        
        self.setup_ui()
        
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
        
        # Bag文件文件夹选择
        ttk.Label(main_frame, text="Bag文件文件夹:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.bag_folder_var = tk.StringVar()
        bag_folder_frame = ttk.Frame(main_frame)
        bag_folder_frame.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=5)
        self.bag_folder_entry = ttk.Entry(bag_folder_frame, textvariable=self.bag_folder_var, width=50)
        self.bag_folder_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(bag_folder_frame, text="浏览...", command=self.select_bag_folder).pack(side=tk.LEFT, padx=5)
        
        # 车辆编号
        ttk.Label(main_frame, text="车辆编号:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.vehicle_id_var = tk.StringVar(value="F520MR")
        self.vehicle_id_entry = ttk.Entry(main_frame, textvariable=self.vehicle_id_var)
        self.vehicle_id_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5)
        
        # 输出文件夹
        ttk.Label(main_frame, text="输出文件夹:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.output_folder_var = tk.StringVar(value="2025_10_05")
        self.output_folder_entry = ttk.Entry(main_frame, textvariable=self.output_folder_var)
        self.output_folder_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=5)
        
        # 文件后缀
        ttk.Label(main_frame, text="文件后缀:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.file_suffix_var = tk.StringVar(value="GT")
        self.file_suffix_entry = ttk.Entry(main_frame, textvariable=self.file_suffix_var)
        self.file_suffix_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=5)
        
        # 过滤零值选项
        self.filter_zeros_var = tk.BooleanVar(value=False)
        self.filter_zeros_check = ttk.Checkbutton(
            main_frame, 
            text="只保存非零字段",
            variable=self.filter_zeros_var
        )
        self.filter_zeros_check.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # 进度条
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(main_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # 按钮框架
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=6, column=0, columnspan=2, pady=10)
        
        self.start_button = ttk.Button(button_frame, text="开始处理", command=self.start_processing)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="停止", command=self.stop_processing, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        # 日志区域
        ttk.Label(main_frame, text="处理日志:").grid(row=7, column=0, sticky=tk.W, pady=(10, 0))
        
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=8, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # 配置网格权重使日志区域可扩展
        main_frame.rowconfigure(8, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 添加垂直滚动条
        log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text['yscrollcommand'] = log_scrollbar.set
        
    def select_bag_folder(self):
        """选择Bag文件文件夹"""
        folder = filedialog.askdirectory(title="选择Bag文件文件夹")
        if folder:
            self.bag_folder_var.set(folder)
    
    def log(self, message):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"{timestamp} - {message}"
        self.log_queue.put(log_message)
    
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
    
    def start_processing(self):
        """开始处理"""
        bag_folder = self.bag_folder_var.get().strip()
        vehicle_id = self.vehicle_id_var.get().strip()
        output_folder = self.output_folder_var.get().strip()
        file_suffix = self.file_suffix_var.get().strip()
        filter_zeros = self.filter_zeros_var.get()
        
        if not bag_folder:
            messagebox.showwarning("警告", "请选择Bag文件文件夹!")
            return
        
        if not vehicle_id:
            messagebox.showwarning("警告", "请输入车辆编号!")
            return
        
        if not output_folder:
            output_folder = "GT"
        
        # 更新UI状态
        self.processing = True
        self.stop_requested = False
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.progress_var.set(0)
        
        # 在新线程中处理
        thread = threading.Thread(
            target=self.process_bag_folder,
            args=(bag_folder, vehicle_id, output_folder, file_suffix, filter_zeros)
        )
        thread.daemon = True
        thread.start()
    
    def stop_processing(self):
        """停止处理"""
        if self.processing:
            self.stop_requested = True
            self.log("正在停止处理...")
    
    def process_bag_folder(self, bag_folder, vehicle_id, output_folder, file_suffix, filter_zeros):
        """处理文件夹中的所有bag文件"""
        try:
            # 检查文件夹是否存在
            if not os.path.exists(bag_folder) or not os.path.isdir(bag_folder):
                self.log(f"错误: 文件夹路径 {bag_folder} 不存在或不是目录")
                self.on_processing_complete()
                return
            
            # 查找所有.bag文件
            bag_files = []
            for file in os.listdir(bag_folder):
                if file.endswith('.bag'):
                    bag_files.append(os.path.join(bag_folder, file))
            
            if not bag_files:
                self.log(f"警告: 在文件夹 {bag_folder} 中未找到.bag文件")
                self.on_processing_complete()
                return
            
            self.log(f"找到 {len(bag_files)} 个bag文件")
            if filter_zeros:
                self.log("启用零值过滤: 只保存非零字段")
            
            # 处理每个bag文件
            for i, bag_file in enumerate(bag_files):
                if self.stop_requested:
                    self.log("处理已停止")
                    break
                
                self.log(f"处理文件: {os.path.basename(bag_file)}")
                self.process_bag(bag_file, vehicle_id, output_folder, file_suffix, filter_zeros)
                
                # 更新进度
                progress = (i + 1) / len(bag_files) * 100
                self.progress_var.set(progress)
            
            if not self.stop_requested:
                self.log(f"完成! 共处理 {len(bag_files)} 个bag文件")
            
        except Exception as e:
            self.log(f"处理过程中出现错误: {str(e)}")
        finally:
            self.on_processing_complete()
    
    def has_non_zero_fields(self, row):
        """检查行中是否有非零字段（除了timestamp）"""
        # 跳过timestamp字段（第一个字段）
        for value in row[1:]:
            try:
                if float(value) != 0:
                    return True
            except ValueError:
                # 如果转换失败，假设是非零值
                return True
        return False
    
    def get_message_field(self, msg, field_name):
        """通用方式获取消息字段值，并处理速度单位转换"""
        try:
            if hasattr(msg, field_name):
                value = getattr(msg, field_name)
            elif hasattr(msg, 'header') and hasattr(msg.header, field_name):
                value = getattr(msg.header, field_name)
            elif hasattr(msg, '__slots__'):
                for slot in msg.__slots__:
                    if slot == field_name:
                        value = getattr(msg, slot)
                        break
                else:
                    value = 0
            else:
                value = 0
            
            speed_fields = {
                'Target1SpeedKPH',  
                'Target1LngSpeedKPH',
                'Ego1SpeedKPH'
            }
            
            if field_name in speed_fields:
                try:
                    value = float(value) / 3.6
                except (ValueError, TypeError):
                    value = 0
            
            return value
            
        except Exception as e:
            return 0

    def process_bag(self, bag_path, vehicle_id, output_folder, file_suffix, filter_zeros):
        """处理单个bag文件"""
        try:
            bag_name = os.path.splitext(os.path.basename(bag_path))[0]
            
            # 创建输出目录 - 改为IMUs
            imu_dir = "IMUs"
            full_output_dir = os.path.join(imu_dir, output_folder)
            os.makedirs(full_output_dir, exist_ok=True)
            
            # 生成CSV文件名
            csv_filename = f"{bag_name}_{file_suffix}.csv"
            csv_path = os.path.join(full_output_dir, csv_filename)
            
            self.log(f"创建CSV文件: {csv_path}")
            
            # 打开bag文件
            bag = rosbag.Bag(bag_path, 'r')
            
            # 初始化CSV文件
            csv_file = open(csv_path, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            
            # 写入表头
            csv_writer.writerow(list(self.field_mapping.keys()))
            
            message_count = 0
            saved_count = 0
            
            # 读取/wf/imu_data/parsed话题的消息
            for topic, msg, t in bag.read_messages(topics=['/wf/imu_data/parsed']):
                if self.stop_requested:
                    break
                
                message_count += 1
                
                # 构建数据行
                row = []
                for csv_field, msg_field in self.field_mapping.items():
                    if csv_field == 'dut_timestamp':
                        # 特殊处理时间戳
                        row.append(f"{t.to_sec():.6f}")
                    elif msg_field == 'timestamp':
                        # 从消息的header中获取时间戳
                        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                            row.append(f"{msg.header.stamp.to_sec():.6f}")
                        else:
                            row.append(f"{t.to_sec():.6f}")
                    else:
                        # 通用方式获取字段值
                        value = self.get_message_field(msg, msg_field)
                        # 尝试转换为 float 并保留最多5位小数
                        try:
                            num = float(value)
                            rounded_value = round(num, 5)
                            row.append(rounded_value)
                        except (ValueError, TypeError):
                            row.append(0)
                
                # 根据过滤选项决定是否保存
                should_save = True
                if filter_zeros:
                    should_save = self.has_non_zero_fields(row)
                
                if should_save:
                    csv_writer.writerow(row)
                    saved_count += 1
                
                # 每处理100条消息打印一次进度
                if message_count % 1000 == 0:
                    self.log(f"  已处理 {message_count} 条消息，保存 {saved_count} 条")
            
            csv_file.close()
            bag.close()
            
            self.log(f"完成文件: {os.path.basename(bag_path)}")
            self.log(f"  总共处理: {message_count} 条消息")
            self.log(f"  实际保存: {saved_count} 条消息")
            if filter_zeros:
                self.log(f"  过滤掉: {message_count - saved_count} 条全零消息")
            
        except Exception as e:
            self.log(f"处理文件 {bag_path} 时出错: {str(e)}")
            try:
                if 'csv_file' in locals():
                    csv_file.close()
                if 'bag' in locals():
                    bag.close()
            except:
                pass
    
    def on_processing_complete(self):
        """处理完成后的回调"""
        self.processing = False
        self.root.after(0, self.update_ui_state)
    
    def update_ui_state(self):
        """更新UI状态"""
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.progress_var.set(0)

def main():
    """主函数"""
    
    # 创建GUI
    root = tk.Tk()
    app = ImuToCSVApp(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()
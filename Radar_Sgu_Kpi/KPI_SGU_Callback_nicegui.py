#!/usr/bin/env python3
import asyncio
from pathlib import Path
import rosbag
import numpy as np
import csv
import os
from nicegui import ui
import logging
import queue
from typing import List, Dict

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

RADAR_TOPICS = [
    ("/wf/objectlist_1", "FL"),
    ("/wf/objectlist_2", "FR"),
    ("/wf/objectlist_3", "RL"),
    ("/wf/objectlist_4", "RR"),
]

IMU_TOPIC = "/wf/imu_data/parsed"
DEFAULT_FOLDER = "/home/zjh/桌面/test_rviz_bag/BYD_KPI_recall"

class TimeStampMatcher:
    def __init__(self, log_queue: queue.Queue = None):
        self.log_queue = log_queue or queue.Queue()
        self.processing = False
        self.bag_files = []
        self.folder_path = ""
        
    def log(self, message: str):
        if self.log_queue:
            self.log_queue.put(message)
    
    async def process_files(self, progress_callback=None):
        try:
            self.processing = True
            
            if not self.bag_files:
                self.log("没有找到任何 .bag 文件")
                return
            
            self.log(f"找到 {len(self.bag_files)} 个bag文件")
            
            output_dir = Path("Matched_Results_Recall")
            output_dir.mkdir(exist_ok=True)
            
            for i, bag_file in enumerate(self.bag_files, 1):
                if progress_callback:
                    await progress_callback(i, len(self.bag_files))
                
                bag_name = Path(bag_file).name
                self.log(f"\n[{i}/{len(self.bag_files)}] 处理: {bag_name}")
                
                bag_output_dir = output_dir / Path(bag_file).stem
                bag_output_dir.mkdir(exist_ok=True)
                
                imu_data = self.extract_imu_data(bag_file, bag_name)
                
                if not imu_data:
                    self.log(f"{bag_name}: 没有提取到IMU数据")
                    continue
                
                csv_files = []
                for topic, radar_name in RADAR_TOPICS:
                    self.log(f"处理雷达: {radar_name}")
                    
                    csv_file = self.process_radar_topic(
                        bag_file, bag_name, topic, radar_name, 
                        imu_data, bag_output_dir
                    )
                    
                    if csv_file:
                        csv_files.append(csv_file)
                
                if csv_files:
                    self.log(f"生成 {len(csv_files)} 个CSV文件")
                
                await asyncio.sleep(0.1)
            
            self.log("所有文件处理完成")
            
        except Exception as e:
            self.log(f"处理过程中出错: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            self.processing = False
    
    def extract_imu_data(self, bag_file: str, bag_name: str) -> List[Dict]:
        imu_data = []
        
        try:
            bag = rosbag.Bag(bag_file, 'r')
            imu_count = 0
            
            for topic, msg, t in bag.read_messages(topics=[IMU_TOPIC]):
                imu_record = {
                    'bag_file': bag_name,
                    'timestamp': t.to_sec(),
                }
                
                if t.to_sec() > 1780282077:  # 2026年6月1日的时间戳 1780282077
                    bag.close()
                    raise Exception(f"提取IMU数据时ERROR - TIME EXPIRED -{t.to_sec()} > 2026年6月")
                
                imu_fields = {
                    'Target1CoorX': 'GT_LngDist',
                    'Target1CoorY': 'GT_LatDist',
                    'Target1LngSpeedKPH': 'GT_LngSpeed',
                    'Target1LatSpeedms': 'GT_LatSpeed',
                    'Target1AcceX': 'AcceX_GT',
                    'Target1AcceY': 'AcceY_GT',
                    'Target1HeadingDiff': 'YawAng_GT',
                }
                
                for msg_field, csv_field in imu_fields.items():
                    value = self.get_imu_message_field(msg, msg_field)
                    
                    if msg_field == 'Target1LngSpeedKPH' and value is not None:
                        value = float(value) / 3.6
                    
                    if msg_field == 'Target1HeadingDiff' and value is not None:
                        value = float(value) * (3.1416 / 180.0)
                    
                    imu_record[csv_field] = float(value) if value is not None else 0.0
                
                imu_data.append(imu_record)
                imu_count += 1
            
            bag.close()
            self.log(f"提取IMU数据: {imu_count} 条")
            
        except Exception as e:
            self.log(f"提取IMU数据出错: {e}")
        
        return imu_data
    
    def get_imu_message_field(self, msg, field_name: str):
        try:
            if hasattr(msg, field_name):
                return getattr(msg, field_name)
            
            if hasattr(msg, 'header'):
                header = msg.header
                if hasattr(header, field_name):
                    return getattr(header, field_name)
            
            return None
        except:
            return None
    
    def process_radar_topic(self, bag_file: str, bag_name: str, topic: str, 
                           radar_name: str, imu_data: List[Dict], 
                           output_dir: Path):
        try:
            bag = rosbag.Bag(bag_file, 'r')
            radar_targets = []
            frame_count = 0
            
            for msg_topic, msg, t in bag.read_messages(topics=[topic]):
                if not hasattr(msg, 'ObjectsBuffer'):
                    continue
                
                frame_count += 1
                timestamp = t.to_sec()
                
                for obj_idx, obj in enumerate(msg.ObjectsBuffer):
                    target_info = {
                        'bag_file': bag_name,
                        'radar_name': radar_name,
                        'topic': topic,
                        'frame_id': frame_count,
                        'target_index': obj_idx + 1,
                        'timestamp': timestamp,
                        'sgu_num': len(msg.ObjectsBuffer),
                        
                        'RxReal': obj.RxReal if hasattr(obj, 'RxReal') else 0.0,
                        'RyReal': obj.RyReal if hasattr(obj, 'RyReal') else 0.0,
                        'velAbsX': obj.velAbsX if hasattr(obj, 'velAbsX') else 0.0,
                        'velAbsY': obj.velAbsY if hasattr(obj, 'velAbsY') else 0.0,
                        'accelAbsX': obj.accelAbsX if hasattr(obj, 'accelAbsX') else 0.0,
                        'accelAbsY': obj.accelAbsY if hasattr(obj, 'accelAbsY') else 0.0,
                        'yaw_ang': (obj.yawAng if hasattr(obj, 'yawAng') else 0.0) * (3.1416 / 180.0),
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
                self.log(f"处理 {frame_count} 帧, 提取 {len(radar_targets)} 个目标")
                
                csv_file = self.match_and_save_radar_data(
                    bag_name, radar_name, radar_targets, imu_data, output_dir
                )
                return csv_file
            
        except Exception as e:
            self.log(f"处理雷达 {radar_name} 出错: {e}")
        
        return None
    
    def match_and_save_radar_data(self, bag_name: str, radar_name: str, 
                                radar_targets: List[Dict], imu_data: List[Dict], 
                                output_dir: Path):
        if not radar_targets or not imu_data:
            return None
        
        imu_timestamps = np.array([imu['timestamp'] for imu in imu_data])
        matched_records = []
        match_count = 0
        
        for target in radar_targets:
            idx = np.argmin(np.abs(imu_timestamps - target['timestamp']))
            nearest_imu = imu_data[idx]
            time_diff = abs(nearest_imu['timestamp'] - target['timestamp'])
            
            if time_diff < 0.1:
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
                    
                    'GT_LngDist': nearest_imu.get('GT_LngDist', 0),
                    'GT_LatDist': nearest_imu.get('GT_LatDist', 0),
                    'GT_LngSpeed': nearest_imu.get('GT_LngSpeed', 0),
                    'GT_LatSpeed': nearest_imu.get('GT_LatSpeed', 0),
                    'AcceX_GT': nearest_imu.get('AcceX_GT', 0),
                    'AcceY_GT': nearest_imu.get('AcceY_GT', 0),
                    'YawAng_GT': nearest_imu.get('YawAng_GT', 0),
                    
                    'time_diff': time_diff,
                }
                
                matched_records.append(matched_record)
                match_count += 1
        
        if match_count > 0:
            csv_filename = f"{Path(bag_name).stem}_{radar_name}.csv"
            csv_path = output_dir / csv_filename
            
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
            
            self.log(f"保存: {csv_filename} ({len(matched_records)} 行)")
            return str(csv_path)
        
        return None
@ui.page('/')  # 定义Web应用的根路径页面
def main_page():
    # 创建队列用于线程间通信（主线程和工作线程之间传递日志消息）
    log_queue = queue.Queue()
    # 创建数据处理核心类实例
    matcher = TimeStampMatcher(log_queue)
    
    # 创建页面标题标签
    ui.label("雷达-IMU 时间戳匹配工具(您必须手动输入文件夹地址)").classes("text-h4")
    
    # 创建输入行容器，使用flex布局
    with ui.row().classes("w-full items-center mt-6"):
        ui.label("bag文件-文件夹:").classes("mr-4") 
        
        folder_input = ui.input(value=DEFAULT_FOLDER).props("outlined dense").classes("grow")
        
        ui.button("扫描文件夹", on_click=lambda: find_bag_files())
    
    # 创建文件列表显示卡片
    with ui.card().classes("w-full mt-6"):
        ui.label("找到的文件").classes("text-h6") 
        file_container = ui.column().classes("w-full")
    
    # 创建操作按钮行
    with ui.row().classes("w-full mt-6"):
        start_button = ui.button("开始处理", on_click=lambda: start_processing(), color="positive")
        clear_button = ui.button("清除日志", on_click=lambda: log_area.set_value(""), color="warning")
    
    progress_label = ui.label("在开始处理前，您必须先键入文件夹地址，然后点击扫描文件夹按钮").classes("text-center text-base")
    
    # 日志显示区域标题
    ui.label("处理日志:").classes("text-2xl mt-2")
    log_area = ui.textarea().classes("w-full").props("readonly")
    log_area._props["rows"] = 15
    log_area._props["input-style"] = "font-family: 'Consolas', monospace; font-size: 12px; background-color: #f5f5f5;"
    
    def find_bag_files():
        """扫描文件夹查找bag文件的函数"""
        folder_path = folder_input.value  # 获取输入框的路径
        
        # 验证路径有效性
        if not folder_path or not os.path.isdir(folder_path):
            ui.notify("路径不存在或不是文件夹", type='warning')  # 显示警告通知
            return
        
        try:
            # 遍历文件夹查找所有.bag文件
            bag_files = []
            for root, dirs, files in os.walk(folder_path):
                for file in sorted(files):  # 按文件名排序
                    if file.endswith('.bag'):
                        bag_path = os.path.join(root, file)
                        bag_files.append(bag_path)
            
            file_container.clear()
            
            if bag_files:
                with file_container:
                    for i, file in enumerate(bag_files[:5], 1):
                        ui.label(f"{i}. {os.path.basename(file)}").classes("text-caption")
                    
                    if len(bag_files) > 5:
                        ui.label(f"... 还有 {len(bag_files)-5} 个文件").classes("text-caption text-grey")
                
                # 将找到的文件存储到matcher实例中
                matcher.bag_files = bag_files
                matcher.folder_path = folder_path
                
                ui.notify(f"找到 {len(bag_files)} 个bag文件", type='positive')
                start_button.enable()  
            else:
                ui.notify("没有找到bag文件", type='warning')
                start_button.disable() 
                
        except Exception as e:
            ui.notify(f"查找文件出错: {e}", type='error')
            start_button.disable()
    
    async def start_processing():
        """开始处理的主异步函数"""
        
        log_area.value = ""
        
        start_button.disable()
        clear_button.disable()

        progress_label.set_text("准备开始...")
        
        async def update_progress(current: int, total: int):
            """更新进度的回调函数"""
            progress_label.set_text(f"处理中: {current}/{total} ({current/total*100:.1f}%)")
        
        log_area.value += "=" * 50 + "\n"
        log_area.value += "开始时间戳匹配...\n"
        
        # 创建日志更新任务
        log_updater_task = asyncio.create_task(update_log())
        
        try:
            # 调用核心处理函数
            await matcher.process_files(update_progress)
        finally:
            # 无论成功失败，都取消日志更新任务
            log_updater_task.cancel()
            
            progress_label.set_text("处理完成")

            start_button.enable()
            clear_button.enable()

            ui.notify("处理完成", type='positive', timeout=5000)
    
    async def update_log():
        """持续更新日志区域的异步任务"""
        while True:
            try:
                message = log_queue.get_nowait()
                current_log = log_area.value
                log_area.value = current_log + message + "\n"
            except queue.Empty:
                await asyncio.sleep(0.1)
    
    start_button.disable()
    

# 应用启动入口
if __name__ in {"__main__", "__mp_main__"}:
    # 设置页面标题
    ui.page_title("雷达-IMU 时间戳匹配工具")
    
    # 添加自定义CSS样式
    ui.add_head_html("""
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            background-color: #f5f5f5;
            margin: 0;
            padding: 20px;
        }
        .nicegui-content {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 24px;
            border-radius: 12px;
            box-shadow: 0 2px 12px rgba(0,0,0,0.1);
        }
    </style>
    """)
    
    # 启动Web应用服务器
    ui.run(
        title="雷达-IMU 时间戳匹配工具",  # 浏览器标签页标题
        port=8088,  # 监听端口
        reload=False,  # 关闭热重载
        host='localhost',
    )
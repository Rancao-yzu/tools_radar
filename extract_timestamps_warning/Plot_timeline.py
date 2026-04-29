import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import numpy as np

class SimpleCSVExporter:
    def __init__(self, root):
        self.root = root
        self.root.title("CSV可视化导出工具")
        self.root.geometry("500x400")
        
        # 数据存储
        self.df = None
        
        # 颜色映射
        self.source_colors = {
            'GT': '#FF0000',  # 红色
            'WF': "#ECC384"   # 灰黑色
        }
        
        # 创建界面
        self.create_widgets()
        
    def create_widgets(self):
        """创建界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        title_label = ttk.Label(main_frame, text="警告时间间隔可视化导出工具", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=(0, 20))
        
        # CSV文件选择区域
        file_frame = ttk.Frame(main_frame)
        file_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(file_frame, text="CSV文件:").pack(side=tk.LEFT, padx=(0, 10))
        
        self.csv_path_var = tk.StringVar()
        csv_entry = ttk.Entry(file_frame, textvariable=self.csv_path_var, width=40)
        csv_entry.pack(side=tk.LEFT, padx=(0, 10))
        
        browse_btn = ttk.Button(file_frame, text="浏览...", command=self.browse_csv)
        browse_btn.pack(side=tk.LEFT)
        
        # 加载按钮
        load_btn = ttk.Button(main_frame, text="加载CSV文件", command=self.load_data)
        load_btn.pack(pady=10)
        
        # 导出按钮
        export_btn = ttk.Button(main_frame, text="导出所有图表", command=self.export_all_charts)
        export_btn.pack(pady=10)
        
        # 状态标签
        self.status_var = tk.StringVar(value="就绪")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, 
                                relief=tk.SUNKEN, anchor=tk.CENTER)
        status_label.pack(fill=tk.X, pady=(20, 0))
        
    def browse_csv(self):
        """浏览并选择CSV文件"""
        filepath = filedialog.askopenfilename(
            title="选择CSV文件",
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")]
        )
        if filepath:
            self.csv_path_var.set(filepath)
            
    def load_data(self):
        """加载CSV数据"""
        csv_path = self.csv_path_var.get()
        if not csv_path or not os.path.exists(csv_path):
            messagebox.showerror("错误", "请选择有效的CSV文件路径")
            return
            
        try:
            self.df = pd.read_csv(csv_path)
            
            # 检查必要的列
            required_columns = ['bag', 'warning_type', 'source', 'start_time', 'end_time', 'duration_sec']
            missing_columns = [col for col in required_columns if col not in self.df.columns]
            
            if missing_columns:
                messagebox.showerror("错误", f"CSV文件缺少必要的列:\n{', '.join(missing_columns)}")
                self.df = None
                return
                
            bags_count = len(self.df['bag'].unique())
            self.status_var.set(f"加载成功: {len(self.df)} 行记录, {bags_count} 个bag文件")
            
        except Exception as e:
            messagebox.showerror("错误", f"加载CSV文件时出错:\n{str(e)}")
            
    def export_all_charts(self):
        """导出所有图表到文件"""
        if self.df is None or self.df.empty:
            messagebox.showwarning("警告", "请先加载CSV文件")
            return
            
        # 选择输出目录
        output_dir = filedialog.askdirectory(title="选择输出目录")
        if not output_dir:
            return
            
        try:
            bags = self.df['bag'].unique()
            total_bags = len(bags)
            
            for i, bag_name in enumerate(bags, 1):
                self.status_var.set(f"正在导出图表 {i}/{total_bags}: {bag_name}")
                self.root.update()  # 更新界面显示状态
                
                # 筛选当前bag的数据
                bag_data = self.df[self.df['bag'] == bag_name].copy()
                bag_data = bag_data.sort_values('start_time')
                
                # 创建图形
                fig, ax = plt.subplots(figsize=(16, 10))
                
                # 设置y轴位置
                y_positions = {}
                y_counter = 0
                
                # 为每个warning_type分配y轴位置
                warning_types = bag_data['warning_type'].unique()
                for j, wt in enumerate(sorted(warning_types)):
                    y_positions[wt] = y_counter
                    y_counter += 1
                    
                # 绘制每个报警时间段
                legend_handles = []
                legend_labels = []
                
                # 先绘制GT的数据
                gt_data = bag_data[bag_data['source'] == 'GT']
                for _, row in gt_data.iterrows():
                    y_pos = y_positions[row['warning_type']]
                    start_time = row['start_time']
                    duration = row['duration_sec']
                    
                    # 绘制时间段矩形 - GT用红色
                    rect = plt.Rectangle(
                        (start_time, y_pos - 0.2),
                        duration, 0.4,
                        color=self.source_colors['GT'],
                        alpha=0.8,
                        edgecolor='black',
                        linewidth=1
                    )
                    ax.add_patch(rect)
                    
                    # 添加文本标签
                    label_text = f"{row['warning_type']} (GT)\n{duration:.2f}s"
                    ax.text(
                        start_time + duration/2, y_pos,
                        label_text,
                        ha='center', va='center',
                        fontsize=9, fontweight='bold',
                        color='white'
                    )
                    
                # 再绘制WF的数据
                wf_data = bag_data[bag_data['source'] == 'WF']
                for _, row in wf_data.iterrows():
                    y_pos = y_positions[row['warning_type']] - 0.6
                    start_time = row['start_time']
                    duration = row['duration_sec']
                    
                    # 绘制时间段矩形 - WF用灰黑色
                    rect = plt.Rectangle(
                        (start_time, y_pos - 0.2),
                        duration, 0.4,
                        color=self.source_colors['WF'],
                        alpha=0.8,
                        edgecolor='white',
                        linewidth=1,
                        linestyle='-'
                    )
                    ax.add_patch(rect)
                    
                    # 添加文本标签
                    label_text = f"{row['warning_type']} (WF)\n{duration:.2f}s"
                    ax.text(
                        start_time + duration/2, y_pos,
                        label_text,
                        ha='center', va='center',
                        fontsize=9, fontweight='bold',
                        color='white'
                    )
                    
                # 设置图形属性
                ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
                ax.set_ylabel('Warning Type', fontsize=12, fontweight='bold')
                ax.set_title(f'Warning Time Intervals - {bag_name}', fontsize=14, fontweight='bold', pad=20)
                
                # 设置y轴刻度和标签
                y_ticks = []
                y_labels = []
                for wt, y_pos in y_positions.items():
                    y_ticks.append(y_pos - 0.3)
                    y_labels.append(wt)
                    
                ax.set_yticks(y_ticks)
                ax.set_yticklabels(y_labels, fontsize=11)
                
                # 设置x轴范围
                all_times = np.concatenate([bag_data['start_time'].values, bag_data['end_time'].values])
                time_min = all_times.min() - 1
                time_max = all_times.max() + 1
                ax.set_xlim(time_min, time_max)
                
                # 设置y轴范围
                y_min = -1
                y_max = len(warning_types) - 0.2
                ax.set_ylim(y_min, y_max)
                
                # 添加网格
                ax.grid(True, alpha=0.3, linestyle='--', color='gray')
                ax.set_axisbelow(True)
                
                # 添加图例
                gt_patch = mpatches.Patch(color=self.source_colors['GT'], alpha=0.8, label='GT (Ground Truth)')
                wf_patch = mpatches.Patch(color=self.source_colors['WF'], alpha=0.8, label='WF (Workflow)')
                legend_handles.extend([gt_patch, wf_patch])
                legend_labels.extend(['GT (Ground Truth)', 'WF (Workflow)'])
                
                # 添加warning_type图例
                warning_type_patches = []
                warning_type_labels = []
                for wt in sorted(warning_types):
                    patch = mpatches.Patch(color='gray', alpha=0.3, label=f'{wt}')
                    warning_type_patches.append(patch)
                    warning_type_labels.append(wt)
                    
                # 合并图例
                all_handles = legend_handles + warning_type_patches
                all_labels = legend_labels + warning_type_labels
                
                ax.legend(handles=all_handles, labels=all_labels, 
                          loc='upper right', bbox_to_anchor=(1.15, 1), fontsize=10,
                          title="Legend", title_fontsize=11)
                
                # 添加总时间信息
                total_duration = time_max - time_min
                time_range_str = f"Time Range: {time_min:.2f}s to {time_max:.2f}s (Total: {total_duration:.2f}s)"
                ax.text(0.5, -0.12, time_range_str, transform=ax.transAxes,
                        ha='center', fontsize=10, fontstyle='italic')
                
                plt.tight_layout()
                
                # 保存图片
                safe_filename = bag_name.replace('.bag', '').replace('/', '_').replace('\\', '_')
                output_path = os.path.join(output_dir, f"{safe_filename}.png")
                plt.savefig(output_path, dpi=300, bbox_inches='tight')
                plt.close()
                
            self.status_var.set(f"完成: 已导出 {total_bags} 个图表到 {output_dir}")
            messagebox.showinfo("完成", f"已成功导出 {total_bags} 个图表到:\n{output_dir}")
            
        except Exception as e:
            messagebox.showerror("错误", f"导出图表时出错:\n{str(e)}")
            self.status_var.set("导出失败")

def main():
    """主函数"""
    root = tk.Tk()
    app = SimpleCSVExporter(root)
    root.mainloop()

if __name__ == "__main__":
    main()
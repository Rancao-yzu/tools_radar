#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
角雷达报警KPI评估工具 - GUI界面
使用tkinter创建简单的文件选择界面
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import sys
import threading


class RadarEvalGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("角雷达报警KPI评估工具 v1.1")
        self.root.geometry("560x500")
        
        # 设置样式
        self.style = ttk.Style()
        self.style.theme_use('alt')
        
        # 创建主框架
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        
        # Bag目录选择
        ttk.Label(main_frame, text="Bag文件目录:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.bag_dir_var = tk.StringVar(value=os.getcwd())
        dir_frame = ttk.Frame(main_frame)
        dir_frame.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5)
        
        self.dir_entry = ttk.Entry(dir_frame, textvariable=self.bag_dir_var, width=40)
        self.dir_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(dir_frame, text="浏览", command=self.browse_dir)
        browse_btn.pack(side=tk.RIGHT, padx=(5, 0))
        
        # 雷达ID选择
        ttk.Label(main_frame, text="GT侧雷达ID:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.radar_id_var = tk.StringVar(value="1")
        radar_combo = ttk.Combobox(main_frame, textvariable=self.radar_id_var, 
                                  values=["1", "2", "3", "4"], state="readonly", width=10)
        radar_combo.grid(row=2, column=1, sticky=tk.W, pady=5)
        
        # WF侧雷达ID
        ttk.Label(main_frame, text="WF侧雷达ID:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.radar_id_wf_var = tk.StringVar(value="4")
        radar_wf_combo = ttk.Combobox(main_frame, textvariable=self.radar_id_wf_var, 
                                     values=["2", "4"], state="readonly", width=10)
        radar_wf_combo.grid(row=3, column=1, sticky=tk.W, pady=5)
        
        # 帧容差设置
        ttk.Label(main_frame, text="帧号容差(±):").grid(row=4, column=0, sticky=tk.W, pady=5)
        self.frame_tol_var = tk.StringVar(value="15")
        ttk.Entry(main_frame, textvariable=self.frame_tol_var, width=10).grid(row=4, column=1, sticky=tk.W, pady=5)
        
        # GT端防抖设置
        ttk.Label(main_frame, text="GT防抖间隙帧数:").grid(row=5, column=0, sticky=tk.W, pady=5)
        self.merge_gap_var = tk.StringVar(value="2")
        ttk.Entry(main_frame, textvariable=self.merge_gap_var, width=10).grid(row=5, column=1, sticky=tk.W, pady=5)
        
        # 按钮区域
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=8, column=0, columnspan=2, pady=20)
        
        self.eval_btn = ttk.Button(button_frame, text="开始评估", command=self.start_evaluation, width=15)
        self.eval_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="退出", command=self.root.quit, width=10).pack(side=tk.LEFT, padx=5)
        
        # 日志区域
        log_frame = ttk.Frame(main_frame, padding="5") 
        log_frame.grid(row=9, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # 创建文本控件和滚动条
        self.log_text = tk.Text(log_frame, height=8, width=60, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=1, column=0, sticky=(tk.W, tk.E))

        self.log_message(f"\
--GT侧雷达ID 1 --WF侧雷达ID 2 ->左前雷达功能\n\
--GT侧雷达ID 2 --WF侧雷达ID 2 ->右前雷达功能\n\
--GT侧雷达ID 3 --WF侧雷达ID 4 ->左后雷达功能\n\
--GT侧雷达ID 4 --WF侧雷达ID 4 ->右后雷达功能\n\
    "
        )
        
    def browse_dir(self):
        """选择bag文件目录"""
        directory = filedialog.askdirectory(initialdir=self.bag_dir_var.get())
        if directory:
            self.bag_dir_var.set(directory)
            
        
    def log_message(self, message):
        """在日志区域添加消息"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.root.update()
        
    def start_evaluation(self):
        """开始评估"""
        # 验证输入
        if not os.path.isdir(self.bag_dir_var.get()):
            messagebox.showerror("错误", f"目录不存在: {self.bag_dir_var.get()}")
            return
            
        # 检查是否有bag文件
        bag_files = [f for f in os.listdir(self.bag_dir_var.get()) if f.lower().endswith(".bag")]
        if not bag_files:
            messagebox.showwarning("警告", f"目录中没有找到.bag文件: {self.bag_dir_var.get()}")
            return
            
        # 验证数值输入
        try:
            frame_tol = int(self.frame_tol_var.get())
            merge_gap = int(self.merge_gap_var.get())
            if frame_tol < 1 or merge_gap < 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("错误", "帧容差和防抖间隙必须为正整数")
            return
                    
        output_dir = os.path.join(os.getcwd(), "OUTKPI")
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception as e:
            messagebox.showerror("错误", f"无法创建输出目录: {e}")
            return
            
        # 禁用开始按钮
        self.eval_btn.config(state=tk.DISABLED)
        self.status_var.set("正在评估...")
        
        # 在后台线程中运行评估
        eval_thread = threading.Thread(target=self.run_evaluation, daemon=True)
        eval_thread.start()
        
    def run_evaluation(self):
        """运行评估逻辑"""
        try:
            self.log_message(f"开始评估...")
            self.log_message(f"Bag目录: {self.bag_dir_var.get()}")
            self.log_message(f"雷达ID: GT={self.radar_id_var.get()}, WF={self.radar_id_wf_var.get()}")
            
            # 运行评估程序
            result = evaluate_radar_metrics(
                bag_dir=self.bag_dir_var.get(),
                radar_id_gt=int(self.radar_id_var.get()),
                radar_id_wf=int(self.radar_id_wf_var.get()),
                frame_tol=int(self.frame_tol_var.get()),
                merge_gap_frames=int(self.merge_gap_var.get())
            )
            
            # 显示输出
            self.log_message(result)
                
            self.log_message("评估完成！")
            self.status_var.set("评估完成")
                
        except Exception as e:
            self.log_message(f"运行错误: {str(e)}")
            self.status_var.set("运行错误")
        finally:
            # 重新启用按钮
            self.root.after(0, lambda: self.eval_btn.config(state=tk.NORMAL))

def main():
    root = tk.Tk()
    app = RadarEvalGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
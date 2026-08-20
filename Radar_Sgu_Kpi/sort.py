#!/usr/bin/env python3
"""
检查 Matched_Results_Recall 下所有 CSV 的 frame_id 是否按序排列 (非递减)。
只检查不修改。用法: python3 sort.py
"""

import csv
import os
import glob


def check_csv(csv_path):
    """检查 frame_id 是否非递减, 返回 (总行数, 乱序位置列表)"""
    with open(csv_path, 'r', newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        ids = [int(r.get('frame_id', 0)) for r in reader]

    bad = []
    for i in range(1, len(ids)):
        if ids[i] < ids[i - 1]:
            bad.append((i, ids[i - 1], ids[i]))
    return len(ids), bad, ids


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    csv_files = glob.glob(os.path.join(base_dir, '**', '*.csv'), recursive=True)

    if not csv_files:
        print("未找到CSV文件")
        return

    print(f"找到 {len(csv_files)} 个CSV文件, 开始检查...\n")
    has_problem = False
    for csv_path in sorted(csv_files):
        rel = os.path.relpath(csv_path, base_dir)
        try:
            total, bad, ids = check_csv(csv_path)
            if bad:
                has_problem = True
                print(f"  [乱序] {rel} (共{total}行)")
                for pos, prev_id, cur_id in bad[:5]:
                    print(f"         行{pos+2}: frame_id {prev_id} -> {cur_id}")
                if len(bad) > 5:
                    print(f"         ... 还有 {len(bad)-5} 处乱序")
            else:
                print(f"  [OK]   {rel} ({total}行, frame_id {min(ids)}-{max(ids)})")
        except Exception as e:
            print(f"  [ERROR] {rel}: {e}")

    print(f"\n检查完成! {'存在乱序问题' if has_problem else '全部正常'}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
測試點位檔案路徑的腳本
用於確認路徑是否正確
"""

import os
import json

def test_points_path():
    """測試點位檔案路徑"""
    print("=== 測試點位檔案路徑 ===")
    
    # 方法1: 相對於當前腳本的路徑
    current_dir = os.path.dirname(os.path.abspath(__file__))
    relative_path = os.path.join(current_dir, "saved_points", "robot_points.json")
    
    print(f"當前腳本目錄: {current_dir}")
    print(f"構建的相對路徑: {relative_path}")
    print(f"相對路徑是否存在: {os.path.exists(relative_path)}")
    
    # 方法2: 您提供的絕對路徑
    absolute_path = r"C:\Users\user\Documents\GitHub\DobotDR\Automation\M1Pro\saved_points\robot_points.json"
    print(f"\n您的絕對路徑: {absolute_path}")
    print(f"絕對路徑是否存在: {os.path.exists(absolute_path)}")
    
    # 檢查saved_points目錄
    saved_points_dir = os.path.join(current_dir, "saved_points")
    print(f"\nsaved_points目錄: {saved_points_dir}")
    print(f"saved_points目錄是否存在: {os.path.exists(saved_points_dir)}")
    
    if os.path.exists(saved_points_dir):
        files = os.listdir(saved_points_dir)
        print(f"saved_points目錄內容: {files}")
    
    # 嘗試載入點位檔案
    test_file = relative_path if os.path.exists(relative_path) else absolute_path
    
    if os.path.exists(test_file):
        print(f"\n嘗試載入點位檔案: {test_file}")
        try:
            with open(test_file, "r", encoding="utf-8") as f:
                points_data = json.load(f)
            
            print(f"載入成功，包含 {len(points_data)} 個點位:")
            for point in points_data:
                print(f"  - {point['name']}")
                
        except Exception as e:
            print(f"載入失敗: {e}")
    else:
        print(f"\n點位檔案不存在於任何測試路徑")
        
        # 建議檢查的路徑
        possible_paths = [
            os.path.join(current_dir, "robot_points.json"),
            os.path.join(current_dir, "..", "saved_points", "robot_points.json"),
            os.path.join(os.getcwd(), "saved_points", "robot_points.json"),
        ]
        
        print("\n檢查其他可能的路徑:")
        for path in possible_paths:
            print(f"  {path}: {os.path.exists(path)}")

if __name__ == "__main__":
    test_points_path()
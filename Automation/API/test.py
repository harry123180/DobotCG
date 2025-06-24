import sys
import time
import random
import cv2
import numpy as np
import statistics

# 導入您現有的相機管理模組
try:
    from camera_manager import initialize_all_cameras, get_image, shutdown_all
    CAMERA_MANAGER_AVAILABLE = True
except ImportError:
    print("❌ 無法導入 camera_manager 模組")
    CAMERA_MANAGER_AVAILABLE = False

class VisionTimingTest:
    def __init__(self, camera_name="cam_3"):
        self.camera_name = camera_name
        
        # 時間記錄
        self.timing_records = []
        
        # 圓形檢測參數
        self.threshold_roundness = 0.85
        self.min_area = 30000  # 最小面積閾值
        
    def initialize_cameras(self):
        """初始化相機系統"""
        if not CAMERA_MANAGER_AVAILABLE:
            print("❌ camera_manager 模組不可用")
            return False
            
        try:
            print("🚀 初始化相機系統...")
            start_time = time.time()
            
            initialize_all_cameras()
            time.sleep(2)  # 等待初始化完成
            
            init_time = time.time() - start_time
            print(f"✅ 相機系統初始化完成，耗時: {init_time:.3f}秒")
            return True
            
        except Exception as e:
            print(f"❌ 相機系統初始化失敗: {e}")
            return False
    
    def capture_image(self):
        """捕獲圖像"""
        capture_start = time.time()
        
        try:
            # 使用您的相機管理器獲取圖像
            raw_bytes = get_image(self.camera_name)
            
            # 轉換為numpy數組 (使用您現有的格式)
            image = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((1944, 2592))
            
            capture_time = time.time() - capture_start
            return image, capture_time
            
        except Exception as e:
            print(f"❌ 捕獲圖像失敗: {e}")
            capture_time = time.time() - capture_start
            return None, capture_time
    
    def is_circle(self, contour, tolerance=0.2):
        """判斷輪廓是否為圓形 (使用您現有的算法)"""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        return 1 - tolerance < circularity < 1 + tolerance
    
    def detect_circles_method1(self, image):
        """檢測圓形 - 方法1: 使用您現有的Canny算法"""
        if image is None:
            return 0, 0
            
        detect_start = time.time()
        
        try:
            # 模糊處理
            blurred = cv2.GaussianBlur(image, (9, 9), 2)
            
            # Canny 邊緣檢測
            edges = cv2.Canny(blurred, 20, 60)
            
            # 輪廓檢測
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 圓形計數
            circle_count = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.is_circle(contour) and area > self.min_area:
                    circle_count += 1
                    
            detect_time = time.time() - detect_start
            return circle_count, detect_time
            
        except Exception as e:
            print(f"❌ 圓形檢測失敗: {e}")
            detect_time = time.time() - detect_start
            return 0, detect_time
    
    def detect_circles_method2(self, image):
        """檢測圓形 - 方法2: 使用改進的形態學算法"""
        if image is None:
            return 0, 0
            
        detect_start = time.time()
        
        try:
            # 直方圖均衡與模糊
            equalized = cv2.equalizeHist(image)
            blurred = cv2.GaussianBlur(equalized, (3, 3), 0)
            
            # 門檻二值化
            _, binary_thresh = cv2.threshold(blurred, 170, 255, cv2.THRESH_BINARY)
            
            # 初始邊緣偵測
            edges_initial = cv2.Canny(binary_thresh, 10, 350)
            
            # 建立形態學kernel
            kernel_dilate = np.ones((13, 13), np.uint8)
            kernel_erode = np.ones((11, 11), np.uint8)
            
            # 輪廓填滿處理
            contours_initial, _ = cv2.findContours(edges_initial, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filled_mask = np.zeros_like(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
            cv2.drawContours(filled_mask, contours_initial, -1, (255, 255, 255), thickness=cv2.FILLED)
            
            # 反轉並進行形態學處理
            inverted_mask = cv2.bitwise_not(filled_mask)
            morphed_mask = cv2.dilate(inverted_mask, kernel_dilate, iterations=1)
            morphed_mask = cv2.erode(morphed_mask, kernel_erode, iterations=1)
            
            # 再次邊緣偵測
            edges_final = cv2.Canny(morphed_mask, 10, 350)
            contours_final, _ = cv2.findContours(edges_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 圓形檢測
            circle_count = 0
            for cnt in contours_final:
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                if perimeter == 0 or area < 30:
                    continue
                    
                roundness = (4 * np.pi * area) / (perimeter ** 2)
                
                if roundness > self.threshold_roundness:
                    circle_count += 1
                    
            detect_time = time.time() - detect_start
            return circle_count, detect_time
            
        except Exception as e:
            print(f"❌ 圓形檢測失敗 (方法2): {e}")
            detect_time = time.time() - detect_start
            return 0, detect_time
    
    def cleanup_cameras(self):
        """清理相機資源"""
        cleanup_start = time.time()
        
        try:
            shutdown_all()
            cleanup_time = time.time() - cleanup_start
            return cleanup_time
            
        except Exception as e:
            print(f"❌ 清理相機資源失敗: {e}")
            cleanup_time = time.time() - cleanup_start
            return cleanup_time
    
    def run_single_test(self, test_index, detection_method=1):
        """執行單次測試"""
        print(f"\n--- 第 {test_index + 1} 次測試 ---")
        
        # 隨機延遲模擬真實觸發
        delay = random.uniform(0.1, 1.0)
        time.sleep(delay)
        
        total_start = time.time()
        
        # 捕獲圖像
        image, capture_time = self.capture_image()
        if image is None:
            print(f"❌ 第 {test_index + 1} 次測試捕獲圖像失敗")
            return None
            
        # 檢測圓形 (使用指定的檢測方法)
        if detection_method == 1:
            circle_count, detect_time = self.detect_circles_method1(image)
        else:
            circle_count, detect_time = self.detect_circles_method2(image)
        
        total_time = time.time() - total_start
        
        # 記錄結果
        result = {
            'test_index': test_index + 1,
            'circle_count': circle_count,
            'capture_time': capture_time,
            'detect_time': detect_time,
            'total_time': total_time,
            'detection_method': detection_method
        }
        
        print(f"✅ 檢測到圓形數量: {circle_count}")
        print(f"📊 拍照耗時: {capture_time:.3f}秒")
        print(f"📊 檢測耗時: {detect_time:.3f}秒")
        print(f"📊 總耗時: {total_time:.3f}秒")
        print(f"📊 使用檢測方法: {detection_method}")
        
        return result
    
    def run_timing_test(self, num_tests=10, detection_method=1):
        """執行完整的耗時測試"""
        print(f"🚀 開始進行 {num_tests} 次視覺辨識流程耗時測試")
        print(f"🎯 使用相機: {self.camera_name}")
        print(f"🔍 檢測方法: {'Canny邊緣檢測' if detection_method == 1 else '形態學算法'}")
        print("=" * 50)
        
        # 初始化相機系統
        if not self.initialize_cameras():
            return
        
        try:
            for i in range(num_tests):
                result = self.run_single_test(i, detection_method)
                if result:
                    self.timing_records.append(result)
                else:
                    print(f"⚠️ 第 {i + 1} 次測試失敗，跳過")
                    
        finally:
            # 清理相機資源
            cleanup_time = self.cleanup_cameras()
            print(f"\n🧹 清理耗時: {cleanup_time:.3f}秒")
            
        # 生成統計報告
        self.generate_report()
    
    def run_comparison_test(self, num_tests=5):
        """執行兩種檢測方法的對比測試"""
        print(f"🚀 開始進行檢測方法對比測試 ({num_tests} 次)")
        print(f"🎯 使用相機: {self.camera_name}")
        print("=" * 50)
        
        # 初始化相機系統
        if not self.initialize_cameras():
            return
        
        try:
            for i in range(num_tests):
                print(f"\n=== 第 {i + 1} 次對比測試 ===")
                
                # 捕獲一次圖像
                image, capture_time = self.capture_image()
                if image is None:
                    print(f"❌ 第 {i + 1} 次測試捕獲圖像失敗")
                    continue
                
                print(f"📊 拍照耗時: {capture_time:.3f}秒")
                
                # 方法1: Canny檢測
                circle_count1, detect_time1 = self.detect_circles_method1(image)
                print(f"🔍 方法1 (Canny): {circle_count1} 個圓形, 耗時 {detect_time1:.3f}秒")
                
                # 方法2: 形態學檢測  
                circle_count2, detect_time2 = self.detect_circles_method2(image)
                print(f"🔍 方法2 (形態學): {circle_count2} 個圓形, 耗時 {detect_time2:.3f}秒")
                
                # 對比分析
                if circle_count1 == circle_count2:
                    print("✅ 兩種方法檢測結果一致")
                else:
                    print(f"⚠️ 檢測結果不一致: 方法1={circle_count1}, 方法2={circle_count2}")
                
                print(f"⚡ 速度對比: 方法1 {'更快' if detect_time1 < detect_time2 else '更慢'}")
                
        finally:
            # 清理相機資源
            cleanup_time = self.cleanup_cameras()
            print(f"\n🧹 清理耗時: {cleanup_time:.3f}秒")
    
    def generate_report(self):
        """生成統計報告"""
        if not self.timing_records:
            print("❌ 沒有有效的測試記錄")
            return
            
        print("\n" + "=" * 60)
        print("📈 測試統計報告")
        print("=" * 60)
        
        # 基本統計
        total_tests = len(self.timing_records)
        circle_counts = [r['circle_count'] for r in self.timing_records]
        capture_times = [r['capture_time'] for r in self.timing_records]
        detect_times = [r['detect_time'] for r in self.timing_records]
        total_times = [r['total_time'] for r in self.timing_records]
        
        print(f"📊 測試配置:")
        print(f"   • 使用相機: {self.camera_name}")
        print(f"   • 檢測方法: {'Canny邊緣檢測' if self.timing_records[0].get('detection_method', 1) == 1 else '形態學算法'}")
        print(f"   • 總測試次數: {total_tests}")
        print(f"   • 成功率: {total_tests}/10 ({total_tests/10*100:.1f}%)")
        
        print(f"\n🎯 圓形檢測結果:")
        print(f"   • 平均檢測到: {statistics.mean(circle_counts):.1f} 個圓形")
        print(f"   • 最多檢測到: {max(circle_counts)} 個圓形")
        print(f"   • 最少檢測到: {min(circle_counts)} 個圓形")
        if len(circle_counts) > 1:
            print(f"   • 標準差: {statistics.stdev(circle_counts):.2f}")
        
        print(f"\n⏱️ 拍照耗時統計:")
        print(f"   • 平均耗時: {statistics.mean(capture_times):.3f} 秒")
        print(f"   • 最長耗時: {max(capture_times):.3f} 秒")
        print(f"   • 最短耗時: {min(capture_times):.3f} 秒")
        if len(capture_times) > 1:
            print(f"   • 標準差: {statistics.stdev(capture_times):.3f} 秒")
        
        print(f"\n🔍 檢測耗時統計:")
        print(f"   • 平均耗時: {statistics.mean(detect_times):.3f} 秒")
        print(f"   • 最長耗時: {max(detect_times):.3f} 秒")
        print(f"   • 最短耗時: {min(detect_times):.3f} 秒")
        if len(detect_times) > 1:
            print(f"   • 標準差: {statistics.stdev(detect_times):.3f} 秒")
        
        print(f"\n⚡ 總流程耗時統計:")
        print(f"   • 平均耗時: {statistics.mean(total_times):.3f} 秒")
        print(f"   • 最長耗時: {max(total_times):.3f} 秒")
        print(f"   • 最短耗時: {min(total_times):.3f} 秒")
        if len(total_times) > 1:
            print(f"   • 標準差: {statistics.stdev(total_times):.3f} 秒")
        
        # 性能評估
        avg_total_time = statistics.mean(total_times)
        fps_estimate = 1.0 / avg_total_time if avg_total_time > 0 else 0
        print(f"\n🚀 性能評估:")
        print(f"   • 理論最大幀率: {fps_estimate:.1f} FPS")
        print(f"   • 單次檢測平均耗時: {avg_total_time:.3f} 秒")
        
        # 詳細結果表格
        print(f"\n📋 詳細測試結果:")
        print("次數 | 圓形數 | 拍照(s) | 檢測(s) | 總計(s) | 方法")
        print("-" * 60)
        for r in self.timing_records:
            method_str = "Canny" if r.get('detection_method', 1) == 1 else "形態學"
            print(f"{r['test_index']:2d}   |   {r['circle_count']:2d}   | {r['capture_time']:6.3f} | {r['detect_time']:6.3f} | {r['total_time']:6.3f} | {method_str}")

def main():
    """主函數"""
    print("🎯 視覺辨識流程耗時測試工具")
    print("=" * 40)
    
    # 設置目標相機名稱 (根據您的camera_manager配置調整)
    camera_name = "cam_3"  # 您可以修改這個相機名稱
    
    # 創建測試實例
    tester = VisionTimingTest(camera_name)
    
    # 選擇測試模式
    print("請選擇測試模式:")
    print("1. 基本測試 (Canny檢測方法)")
    print("2. 基本測試 (形態學檢測方法)")
    print("3. 檢測方法對比測試")
    
    try:
        choice = input("請輸入選擇 (1-3): ").strip()
        
        if choice == "1":
            tester.run_timing_test(10, detection_method=1)
        elif choice == "2":
            tester.run_timing_test(10, detection_method=2)
        elif choice == "3":
            tester.run_comparison_test(5)
        else:
            print("無效選擇，使用默認模式...")
            tester.run_timing_test(10, detection_method=1)
            
    except KeyboardInterrupt:
        print("\n🛑 用戶中斷測試")
    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")

if __name__ == "__main__":
    main()
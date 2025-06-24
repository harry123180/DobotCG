import numpy as np
import os

# 棋盤格左上角基準座標（單位：mm）
x0 = -15.14
y0 = -239.42

# 棋盤格尺寸與間距
cols = 17  # x 方向
rows = 12  # y 方向
spacing = 10  # mm

# 建立點位資料：格式為 [id, x, y]
point_data = []
for j in range(rows):      # Y 方向（從上往下）
    for i in range(cols):  # X 方向（從左往右）
        idx = j * cols + i
        y = y0 - (i * spacing)
        x = x0 - (j * spacing)
        point_data.append([idx, x, y])

# 轉為 numpy array
points_array = np.array(point_data, dtype=np.float32)

# 儲存路徑（與此 Python 檔案在同一層）
save_path = os.path.join(os.path.dirname(__file__), "world_points.npy")
np.save(save_path, points_array)

print(f"已儲存真實世界座標至：{save_path}")

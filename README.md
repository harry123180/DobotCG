# DobotM1Project
## 一些寫給自己的重要訊息

* 機械臂的LAN1 IP是 **192.168.1.6** LAN2 IP是**192.168.2.6**
* Python開發環境3.12.4
* pytorch版本 2.4.1
* 顯示卡型號RTX 4060Ti
* CUDA版本12.1.0
* cuDnn版本9.1.0
* 測試單GPU多模型的執行速度
opencvtest.py 測試完成mks driver done 


# 待辦事項

1. 相機標定
2. 影像流觸發運算



| 機械臂 | 欄位2 | 欄位3 |
| :-- | --: |:--:|
| 置左  | 置右 | 置中 |
# 原始圖像尺寸
2592 x 1944
## 重投影误差 (Reprojection Error)

$$ RE = 0.21439245149643946 $$

## 内参矩阵 (Intrinsic Parameters)
$$
\mathbf{K} =
\begin{bmatrix}
5527.91522 & 0.00000 & 1249.56097 \\
0.00000 & 5523.37409 & 997.41524 \\
0.00000 & 0.00000 & 1.00000
\end{bmatrix}
$$

## 畸变系数 (Distortion Coefficients)
$$
\mathbf{D} =
\begin{bmatrix}
-0.06833483 & 0.00056340 & 0.00137019 & 0.00055740 & 4.80949681
\end{bmatrix}
$$

# 當前檔案結構 
```
├─converted_jpgs
├─DobotDemo
│  ├─files
│  │  └─__pycache__
│  ├─images
│  ├─picture
│  └─__pycache__
├─MVSdemo
│  └─BasicDemo
│      └─__pycache__
└─Mycode
    ├─chessboard <這邊放標記完的棋盤格JPG>
    ├─converted_jpgs <這邊放還沒標記棋盤格JPG>
    ├─datafile  <海康原圖bmp 格式>
    ├─files <不動>
    │  └─__pycache__
    ├─__pycache__
    ├─procces.py <處理內部參數計算>    

```
| 末端執行器     |     DH夾爪 |  DH夾爪     |
| --------      | -------    | -------    |
| 1       | 紅色       |      24V      |
| 2      | 白色       |     INPUT1     |
| 3         | 棕色       |     INPUT2    |    
| 11         | 橙色       |   OUTPUTI   |    
| 12         | 黄色       |   OUTPUT2  |    
| 15         | 黑色       |    GND     |    
| 10         | 藍色       |    485_B     |  
| 9         | 綠色       |    485_A   |  
![alt text](image.png)

![alt text](image-1.png)
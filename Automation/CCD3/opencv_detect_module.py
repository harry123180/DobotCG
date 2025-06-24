import cv2
import numpy as np
import math
import random

#影像前處理
def get_pre_treatment_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

## 角度檢測
def get_main_contour(image,min_area_size_rate = 0.05,sequence = False):
    min_area = image.shape[0]*image.shape[1] * min_area_size_rate
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    if not contours:
        return None
    # 找出最大內輪廓
    if sequence:#
        contour = contours[-1]
    else:
        contour = contours[0]
    return contour

def get_obj_angle(image,mode=0):
    #mode->0 case ,mode->1 dr
    contour = None
    rst_contour = None

    pt_img = get_pre_treatment_image(image)
    
    if mode == 0:
        contour = get_main_contour(pt_img,sequence=True)
        if contour is None:
            return None
        mask_1 = np.zeros((image.shape[1], image.shape[0]), dtype=np.uint8)
        mask_2 = np.zeros((image.shape[1], image.shape[0]), dtype=np.uint8)
        cv2.drawContours(mask_1, [contour], -1, (255,255,255), -1)
        ellipse = cv2.fitEllipse(contour)

        # 取得中心與角度
        (x, y), (MA, ma), angle = ellipse
        print(MA)
        center = (int(x), int(y))
        # cv2.line(image,(center[0],int(center[1] - MA//2)),(center[0],int(center[1] + MA//2)),(10,50,134),2)
        # cv2.putText(image, f"MA: {MA:.2f}", (center[0]-200 , center[1] - 10),
        #         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 200, 0), 2)
        # 畫橢圓與方向
        cv2.ellipse(mask_1, ellipse, (0,0,0), -1)
        # cv2.ellipse(image, ellipse, (35,150,0), 1)
        # print(ellipse)
        center, radius = cv2.minEnclosingCircle(contour)
        center = (int(center[0]),int(center[-1]))
        
        cv2.circle(mask_2,center,int(radius),(255,255,255),-1)
        kernel = np.ones((11, 11), np.uint8)
        mask_1 = cv2.dilate(mask_1,kernel,iterations = 1)
        mask_1 = cv2.bitwise_not(mask_1)
        rst = cv2.bitwise_and(mask_1,mask_1,mask=mask_2)
        rst_contour = get_main_contour(rst)

    else:
        rst_contour = get_main_contour(pt_img)
        if rst_contour is None:
            return None
    rect = cv2.minAreaRect(rst_contour)
    box = cv2.boxPoints(rect)
    box = np.int_(box)
    angle = rect[2]

    # 視覺化
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    center = tuple(np.int_(rect[0]))
    
    return center,angle



# 影片來源
video_path = "video/Video_20250605172016964.avi"

# 重新讀取影片
cap = cv2.VideoCapture(video_path)
# 處理每幀
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame2 = cv2.resize(frame,(600,600))
    center,angle = get_obj_angle(frame2,mode=0)
    # frame2 = 
    cv2.putText(frame2, f"Angle: {angle:.2f} deg", (center[0] - 70, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    # cv2.imshow("main",cv2.resize(frame,(1000,1000)))
    cv2.imshow("main2",frame2)
    if cv2.waitKey(0) & 0xff  == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()







# def detect_angle_with_minAreaRect(video_path):
#     cap = cv2.VideoCapture(video_path)

#     while cap.isOpened():
#         ret, frame = cap.read()
#         if not ret:
#             break

#         contour = get_main_contour(frame)
#         if contour is None:
#             continue

#         # 使用 minAreaRect 擬合外接旋轉矩形
#         rect = cv2.minAreaRect(contour)
#         box = cv2.boxPoints(rect)
#         box = np.int_(box)
#         angle = rect[2]

#         # 視覺化
#         cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
#         center = tuple(np.int_(rect[0]))
#         cv2.putText(frame, f"Angle: {angle:.2f} deg", (center[0] - 70, center[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

#         cv2.imshow("minAreaRect Angle Detection", cv2.resize(frame,(1000,1000)))
#         if cv2.waitKey(0) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()
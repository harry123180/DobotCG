import socket
import threading
from tkinter import Text, END
import datetime
import numpy as np
import os
import json

alarmControllerFile = "files/alarm_controller.json"
alarmServoFile = "files/alarm_servo.json"

# 端口反饋數據結構
MyType = np.dtype([('len', np.int16,),
                   ('Reserve', np.int16, (3,)),
                   ('digital_input_bits', np.int64,),
                   ('digital_outputs', np.int64,),
                   ('robot_mode', np.int64,),
                   ('controller_timer', np.int64,),
                   ('run_time', np.int64,),
                   ('test_value', np.int64,),
                   ('safety_mode', np.float64,),
                   ('speed_scaling', np.float64,),
                   ('linear_momentum_norm', np.float64,),
                   ('v_main', np.float64,),
                   ('v_robot', np.float64,),
                   ('i_robot', np.float64,),
                   ('program_state', np.float64,),
                   ('safety_status', np.float64,),
                   ('tool_accelerometer_values', np.float64, (3,)),
                   ('elbow_position', np.float64, (3,)),
                   ('elbow_velocity', np.float64, (3,)),
                   ('q_target', np.float64, (6,)),
                   ('qd_target', np.float64, (6,)),
                   ('qdd_target', np.float64, (6,)),
                   ('i_target', np.float64, (6,)),
                   ('m_target', np.float64, (6,)),
                   ('q_actual', np.float64, (6,)),
                   ('qd_actual', np.float64, (6,)),
                   ('i_actual', np.float64, (6,)),
                   ('i_control', np.float64, (6,)),
                   ('tool_vector_actual', np.float64, (6,)),
                   ('TCP_speed_actual', np.float64, (6,)),
                   ('TCP_force', np.float64, (6,)),
                   ('Tool_vector_target', np.float64, (6,)),
                   ('TCP_speed_target', np.float64, (6,)),
                   ('motor_temperatures', np.float64, (6,)),
                   ('joint_modes', np.float64, (6,)),
                   ('v_actual', np.float64, (6,)),
                   ('handtype', np.int8, (4,)),
                   ('userCoordinate', np.int8, (1,)),
                   ('toolCoordinate', np.int8, (1,)),
                   ('isRunQueuedCmd', np.int8, (1,)),
                   ('isPauseCmdFlag', np.int8, (1,)),
                   ('velocityRatio', np.int8, (1,)),
                   ('accelerationRatio', np.int8, (1,)),
                   ('jerkRatio', np.int8, (1,)),
                   ('xyzVelocityRatio', np.int8, (1,)),
                   ('rVelocityRatio', np.int8, (1,)),
                   ('xyzAccelerationRatio', np.int8, (1,)),
                   ('rAccelerationRatio', np.int8, (1,)),
                   ('xyzJerkRatio', np.int8, (1,)),
                   ('rJerkRatio', np.int8, (1,)),
                   ('BrakeStatus', np.int8, (1,)),
                   ('EnableStatus', np.int8, (1,)),
                   ('DragStatus', np.int8, (1,)),
                   ('RunningStatus', np.int8, (1,)),
                   ('ErrorStatus', np.int8, (1,)),
                   ('JogStatus', np.int8, (1,)),
                   ('RobotType', np.int8, (1,)),
                   ('DragButtonSignal', np.int8, (1,)),
                   ('EnableButtonSignal', np.int8, (1,)),
                   ('RecordButtonSignal', np.int8, (1,)),
                   ('ReappearButtonSignal', np.int8, (1,)),
                   ('JawButtonSignal', np.int8, (1,)),
                   ('SixForceOnline', np.int8, (1,)),  # 1037
                   ('Reserve2', np.int8, (82,)),
                   ('m_actual[6]', np.float64, (6,)),
                   ('load', np.float64, (1,)),
                   ('centerX', np.float64, (1,)),
                   ('centerY', np.float64, (1,)),
                   ('centerZ', np.float64, (1,)),
                   ('user', np.float64, (6,)),
                   ('tool', np.float64, (6,)),
                   ('traceIndex', np.int64,),
                   ('SixForceValue', np.int64, (6,)),
                   ('TargetQuaternion', np.float64, (4,)),
                   ('ActualQuaternion', np.float64, (4,)),
                   ('Reserve3', np.int8, (24,)),
                   ])


# 讀取控制器和伺服告警檔案
def alarmAlarmJsonFile():
    currrntDirectory = os.path.dirname(__file__)
    jsonContrellorPath = os.path.join(currrntDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currrntDirectory, alarmServoFile)

    with open(jsonContrellorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo


class DobotApi:
    def __init__(self, ip, port, *args):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.__globalLock = threading.Lock()
        self.text_log: Text = None
        if args:
            self.text_log = args[0]

        if self.port == 29999 or self.port == 30003 or self.port == 30004:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
            except socket.error:
                print(socket.error)
                raise Exception(
                    f"無法建立端口 {self.port} 的socket連線！", socket.error)
        else:
            raise Exception(
                f"連接控制面板伺服器需要使用端口 {self.port}！")

    def log(self, text):
        if self.text_log:
            date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S ")
            self.text_log.insert(END, date + text + "\n")
        else:
            print(text)

    def send_data(self, string):
        try:
            #self.log(f"發送至 {self.ip}:{self.port}: {string}")
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)

    def wait_reply(self):
        """
        讀取回傳值
        """
        data = ""
        try:
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
                #self.log(f'接收自 {self.ip}:{self.port}: {data_str}')
            return data_str

    def close(self):
        """
        關閉端口
        """
        if (self.socket_dobot != 0):
            self.socket_dobot.close()

    def sendRecvMsg(self, string):
        """
        發送-接收同步處理
        """
        with self.__globalLock:
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    def __del__(self):
        self.close()


class DobotApiDashboard(DobotApi):
    """
    定義dobot_api_dashboard類別以建立與Dobot的連接
    """

    def EnableRobot(self, *dynParams):
        """
        使能機械臂
        dynParams: 動態參數(負載重量, 偏心X, 偏心Y, 偏心Z)
        """
        string = "EnableRobot("
        for i in range(len(dynParams)):
            if i == len(dynParams) - 1:
                string = string + str(dynParams[i])
            else:
                string = string + str(dynParams[i]) + ","
        string = string + ")"
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        下使能機械臂
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        清除控制器告警訊息
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def ResetRobot(self):
        """
        機械臂停止
        """
        string = "ResetRobot()"
        return self.sendRecvMsg(string)

    def SpeedFactor(self, speed):
        """
        設置全域速度比例
        speed: 速度比例值（範圍：1~100）
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        選擇校準的使用者座標系
        index: 使用者座標系的校準索引
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        選擇校準的工具座標系
        index: 工具座標系的校準索引
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def RobotMode(self):
        """
        查看機械臂狀態
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PayLoad(self, weight, inertia):
        """
        設置機械臂負載
        weight: 負載重量
        inertia: 負載慣量矩
        """
        string = "PayLoad({:f},{:f})".format(weight, inertia)
        return self.sendRecvMsg(string)

    def DO(self, index, status):
        """
        設置數位訊號輸出（隊列指令）
        index: 數位輸出索引（範圍：1~24）
        status: 數位訊號輸出端口狀態（0：低電平，1：高電平）
        """
        string = "DO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        設置關節加速度比例（僅適用於 MovJ, MovJIO, MovJR, JointMovJ 指令）
        speed: 關節加速度比例（範圍：1~100）
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        設置座標系加速度比例（僅適用於 MovL, MovLIO, MovLR, Jump, Arc, Circle 指令）
        speed: 笛卡爾加速度比例（範圍：1~100）
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedJ(self, speed):
        """
        設置關節速度比例（僅適用於 MovJ, MovJIO, MovJR, JointMovJ 指令）
        speed: 關節速度比例（範圍：1~100）
        """
        string = "SpeedJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedL(self, speed):
        """
        設置笛卡爾速度比例（僅適用於 MovL, MovLIO, MovLR, Jump, Arc, Circle 指令）
        speed: 笛卡爾速度比例（範圍：1~100）
        """
        string = "SpeedL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def Arch(self, index):
        """
        設置Jump門型參數索引（包含：起點抬升高度、最大抬升高度、終點下降高度）
        index: 參數索引（範圍：0~9）
        """
        string = "Arch({:d})".format(index)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        設置平滑過渡比例
        ratio: 平滑過渡比例（範圍：1~100）
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def LimZ(self, value):
        """
        設置門型參數的最大抬升高度
        value: 最大抬升高度（高度限制：不得超過機械臂z軸限位）
        """
        string = "LimZ({:d})".format(value)
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
        執行腳本檔案
        project_name: 腳本檔案名稱
        """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def StopScript(self):
        """
        停止腳本執行
        """
        string = "StopScript()"
        return self.sendRecvMsg(string)

    def PauseScript(self):
        """
        暫停腳本執行
        """
        string = "PauseScript()"
        return self.sendRecvMsg(string)

    def ContinueScript(self):
        """
        繼續執行腳本
        """
        string = "ContinueScript()"
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, id, addr, count, type=None):
        """
        讀取保持暫存器
        id: 從設備編號（最多支援5個設備，範圍：0~4，存取控制器內部從站時設為0）
        addr: 保持暫存器起始位址（範圍：3095~4095）
        count: 讀取指定類型數據的數量（範圍：1~16）
        type: 數據類型
            若為空，預設讀取16位無符號整數（2位元組，佔用1個暫存器）
            "U16": 讀取16位無符號整數（2位元組，佔用1個暫存器）
            "U32": 讀取32位無符號整數（4位元組，佔用2個暫存器）
            "F32": 讀取32位單精度浮點數（4位元組，佔用2個暫存器）
            "F64": 讀取64位雙精度浮點數（8位元組，佔用4個暫存器）
        """
        if type is not None:
            string = "GetHoldRegs({:d},{:d},{:d},{:s})".format(
                id, addr, count, type)
        else:
            string = "GetHoldRegs({:d},{:d},{:d})".format(
                id, addr, count)
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, id, addr, count, table, type=None):
        """
        寫入保持暫存器
        id: 從設備編號（最多支援5個設備，範圍：0~4，存取控制器內部從站時設為0）
        addr: 保持暫存器起始位址（範圍：3095~4095）
        count: 寫入指定類型數據的數量（範圍：1~16）
        table: 寫入數據
        type: 數據類型（同GetHoldRegs）
        """
        if type is not None:
            string = "SetHoldRegs({:d},{:d},{:d},{:d})".format(
                id, addr, count, table)
        else:
            string = "SetHoldRegs({:d},{:d},{:d},{:d},{:s})".format(
                id, addr, count, table, type)
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        獲取機械臂錯誤代碼
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

    def DOExecute(self, offset1, offset2):
        """
        立即執行數位輸出設置
        offset1: 數位輸出索引
        offset2: 輸出狀態
        """
        string = "DOExecute({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def ToolDO(self, offset1, offset2):
        """
        設置末端工具數位輸出（隊列指令）
        offset1: 末端數位輸出索引
        offset2: 輸出狀態
        """
        string = "ToolDO({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def ToolDOExecute(self, offset1, offset2):
        """
        立即執行末端工具數位輸出設置
        offset1: 末端數位輸出索引
        offset2: 輸出狀態
        """
        string = "ToolDOExecute({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def SetArmOrientation(self, offset1):
        """
        設置機械臂手系（M1 Pro專用）
        offset1: 手系設置（0：左手系，1：右手系）
        """
        string = "SetArmOrientation({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def SetPayload(self, offset1, *dynParams):
        """
        設置末端負載
        offset1: 負載重量（kg）
        dynParams: 偏心距離（centerX, centerY, centerZ）
        """
        string = "SetPayload({:f}".format(
            offset1)
        for params in dynParams:
            string = string + "," + str(params) + ","
        string = string + ")"
        return self.sendRecvMsg(string)

    def PositiveSolution(self, offset1, offset2, offset3, offset4, user, tool):
        """
        正解運算（關節角度轉笛卡爾座標）
        offset1~4: 關節角度J1~J4
        user: 使用者座標系索引
        tool: 工具座標系索引
        """
        string = "PositiveSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1, offset2, offset3, offset4, user,
                                                                         tool) + ")"
        return self.sendRecvMsg(string)

    def InverseSolution(self, offset1, offset2, offset3, offset4, user, tool, *dynParams):
        """
        逆解運算（笛卡爾座標轉關節角度）
        offset1~4: 笛卡爾座標X,Y,Z,R
        user: 使用者座標系索引
        tool: 工具座標系索引
        dynParams: 額外參數（isJointNear, JointNear）
        """
        string = "InverseSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1, offset2, offset3, offset4, user, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + repr(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, offset1):
        """
        設置碰撞檢測等級
        offset1: 碰撞檢測等級（範圍：0~5）
        """
        string = "SetCollisionLevel({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def GetAngle(self):
        """
        獲取當前關節角度
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self):
        """
        獲取當前笛卡爾座標
        """
        string = "GetPose()"
        return self.sendRecvMsg(string)

    def EmergencyStop(self):
        """
        緊急停止
        """
        string = "EmergencyStop()"
        return self.sendRecvMsg(string)

    def ModbusCreate(self, ip, port, slave_id, isRTU):
        """
        建立Modbus連線
        ip: IP位址
        port: 端口號
        slave_id: 從站ID
        isRTU: 是否為RTU模式（0：TCP，1：RTU）
        """
        string = "ModbusCreate({:s},{:d},{:d},{:d}".format(ip, port, slave_id, isRTU) + ")"
        return self.sendRecvMsg(string)

    def ModbusClose(self, offset1):
        """
        關閉Modbus連線
        offset1: 連線索引
        """
        string = "ModbusClose({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def GetInBits(self, offset1, offset2, offset3):
        """
        讀取離散輸入
        offset1: 連線索引
        offset2: 起始位址
        offset3: 讀取數量
        """
        string = "GetInBits({:d},{:d},{:d}".format(offset1, offset2, offset3) + ")"
        return self.sendRecvMsg(string)

    def GetInRegs(self, offset1, offset2, offset3, *dynParams):
        """
        讀取輸入暫存器
        offset1: 連線索引
        offset2: 起始位址
        offset3: 讀取數量
        dynParams: 數據類型
        """
        string = "GetInRegs({:d},{:d},{:d}".format(offset1, offset2, offset3)
        for params in dynParams:
            print(type(params), params)
            string = string + params[0]
        string = string + ")"
        return self.sendRecvMsg(string)

    def GetCoils(self, offset1, offset2, offset3):
        """
        讀取線圈暫存器
        offset1: 連線索引
        offset2: 起始位址
        offset3: 讀取數量
        """
        string = "GetCoils({:d},{:d},{:d}".format(offset1, offset2, offset3) + ")"
        return self.sendRecvMsg(string)

    def SetCoils(self, offset1, offset2, offset3, offset4):
        """
        寫入線圈暫存器
        offset1: 連線索引
        offset2: 起始位址
        offset3: 寫入數量
        offset4: 寫入數據
        """
        string = "SetCoils({:d},{:d},{:d}".format(offset1, offset2, offset3) + "," + repr(offset4) + ")"
        print(str(offset4))
        return self.sendRecvMsg(string)

    def DI(self, offset1):
        """
        讀取數位輸入狀態
        offset1: 數位輸入索引
        """
        string = "DI({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def ToolDI(self, offset1):
        """
        讀取末端工具數位輸入狀態
        offset1: 末端數位輸入索引
        """
        string = "DI({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def DOGroup(self, *dynParams):
        """
        批量設置數位輸出
        dynParams: 輸出參數（索引1,狀態1,索引2,狀態2...）
        """
        string = "DOGroup("
        for params in dynParams:
            string = string + str(params) + ","
        string = string + ")"
        return self.wait_reply()

    def BrakeControl(self, offset1, offset2):
        """
        抱閘控制
        offset1: 關節索引
        offset2: 抱閘狀態（0：鎖定，1：釋放）
        """
        string = "BrakeControl({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """
        開始拖拽模式
        """
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        """
        停止拖拽模式
        """
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def LoadSwitch(self, offset1):
        """
        負載開關
        offset1: 開關狀態
        """
        string = "LoadSwitch({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def wait(self, t):
        """
        隊列延時
        t: 延時時間（毫秒）
        """
        string = "wait({:d}".format(t)+")"
        return self.sendRecvMsg(string)

    def pause(self):
        """
        暫停運動隊列
        """
        string = "pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
        繼續運動隊列
        """
        string = "continue()"
        return self.sendRecvMsg(string)


class DobotApiMove(DobotApi):
    """
    定義dobot_api_move類別以建立與Dobot的運動控制連線
    """

    def MovJ(self, x, y, z, r, *dynParams):
        """
        關節運動介面（點到點運動模式）
        x: 笛卡爾座標系x座標值
        y: 笛卡爾座標系y座標值
        z: 笛卡爾座標系z座標值
        r: 笛卡爾座標系R旋轉值
        dynParams: 動態參數（User, Tool, SpeedJ, AccJ, CP等）
        """
        string = "MovJ({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def MovL(self, x, y, z, r, *dynParams):
        """
        直線運動介面（線性運動模式）
        x: 笛卡爾座標系x座標值
        y: 笛卡爾座標系y座標值
        z: 笛卡爾座標系z座標值
        r: 笛卡爾座標系R旋轉值
        dynParams: 動態參數（User, Tool, SpeedL, AccL, CP等）
        """
        string = "MovL({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def JointMovJ(self, j1, j2, j3, j4, *dynParams):
        """
        關節座標運動介面
        j1~j4: 各關節位置值
        dynParams: 動態參數（SpeedJ, AccJ, CP等）
        """
        string = "JointMovJ({:f},{:f},{:f},{:f}".format(
            j1, j2, j3, j4)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Jump(self):
        """
        門型運動（待實現）
        """
        print("待實現")

    def RelMovJ(self, x, y, z, r, *dynParams):
        """
        相對運動介面（關節運動模式）
        x: 笛卡爾座標系x軸偏移量
        y: 笛卡爾座標系y軸偏移量
        z: 笛卡爾座標系z軸偏移量
        r: 笛卡爾座標系R軸偏移量
        dynParams: 動態參數
        """
        string = "RelMovJ({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelMovL(self, offsetX, offsetY, offsetZ, offsetR, *dynParams):
        """
        相對運動介面（直線運動模式）
        offsetX: 笛卡爾座標系x軸偏移量
        offsetY: 笛卡爾座標系y軸偏移量
        offsetZ: 笛卡爾座標系z軸偏移量
        offsetR: 笛卡爾座標系R軸偏移量
        dynParams: 動態參數
        """
        string = "RelMovL({:f},{:f},{:f},{:f}".format(offsetX, offsetY, offsetZ, offsetR)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovLIO(self, x, y, z, r, *dynParams):
        """
        直線運動同時設置數位輸出
        x: 笛卡爾座標系x座標值
        y: 笛卡爾座標系y座標值
        z: 笛卡爾座標系z座標值
        r: 笛卡爾座標系R旋轉值
        dynParams: IO參數設置（Mode、Distance、Index、Status）
                Mode: 設置距離模式（0：距離百分比；1：距離數值）
                Distance: 運行指定距離（Mode為0時，範圍0~100；Mode為1時，正值表示起點距離，負值表示終點距離）
                Index: 數位輸出索引（範圍：1~24）
                Status: 數位輸出狀態（範圍：0/1）
        """
        # 範例： MovLIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovLIO({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovJIO(self, x, y, z, r, *dynParams):
        """
        關節運動同時設置數位輸出
        x: 笛卡爾座標系x座標值
        y: 笛卡爾座標系y座標值
        z: 笛卡爾座標系z座標值
        r: 笛卡爾座標系R旋轉值
        dynParams: IO參數設置（同MovLIO）
        """
        # 範例： MovJIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovJIO({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        #self.log("發送至 192.168.1.6:29999:" + string)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Arc(self, x1, y1, z1, r1, x2, y2, z2, r2, *dynParams):
        """
        圓弧運動指令
        x1, y1, z1, r1: 中間點座標值
        x2, y2, z2, r2: 終點座標值
        dynParams: 動態參數
        注意：此指令需與其他運動指令配合使用
        """
        string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f}".format(
            x1, y1, z1, r1, x2, y2, z2, r2)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Circle(self, x1, y1, z1, r1, x2, y2, z2, r2, count, *dynParams):
        """
        整圓運動指令
        x1, y1, z1, r1: 中間點座標值
        x2, y2, z2, r2: 終點座標值
        count: 運行圈數
        dynParams: 動態參數
        注意：此指令需與其他運動指令配合使用
        """
        string = "Circle({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(
            x1, y1, z1, r1, x2, y2, z2, r2, count)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id=None, *dynParams):
        """
        點動控制
        axis_id: 關節運動軸，可選字串值：
            J1+ J2+ J3+ J4+ J5+ J6+
            J1- J2- J3- J4- J5- J6-
            X+ Y+ Z+ Rx+ Ry+ Rz+
            X- Y- Z- Rx- Ry- Rz-
        dynParams: 參數設置（coord_type, user_index, tool_index）
                coord_type: 1：使用者座標 2：工具座標（預設值為1）
                user_index: 使用者索引0~9（預設值為0）
                tool_index: 工具索引0~9（預設值為0）
        """
        if axis_id is not None:
            string = "MoveJog({:s}".format(axis_id)
        else:
            string = "MoveJog("
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def Sync(self):
        """
        阻塞程式執行隊列指令，所有隊列指令執行完畢後返回
        """
        string = "Sync()"
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_r, user, *dynParams):
        """
        沿使用者座標系相對運動指令，終端運動模式為關節運動
        offset_x: X軸方向偏移量
        offset_y: Y軸方向偏移量
        offset_z: Z軸方向偏移量
        offset_r: R軸方向偏移量
        user: 選擇校準的使用者座標系，範圍：0~9
        dynParams: 參數設置（speed_j, acc_j, tool）
                speed_j: 設置關節速度比例，範圍：1~100
                acc_j: 設置加速度比例，範圍：1~100
                tool: 設置工具座標系索引
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_r, user)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_r, user, *dynParams):
        """
        沿使用者座標系相對運動指令，終端運動模式為直線運動
        offset_x: X軸方向偏移量
        offset_y: Y軸方向偏移量
        offset_z: Z軸方向偏移量
        offset_r: R軸方向偏移量
        user: 選擇校準的使用者座標系，範圍：0~9
        dynParams: 參數設置（speed_l, acc_l, tool）
                speed_l: 設置笛卡爾速度比例，範圍：1~100
                acc_l: 設置加速度比例，範圍：1~100
                tool: 設置工具座標系索引
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_r, user)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelJointMovJ(self, offset1, offset2, offset3, offset4, *dynParams):
        """
        沿各軸關節座標系相對運動指令，終端運動模式為關節運動
        offset1~4: 各關節偏移量
        dynParams: 參數設置（speed_j, acc_j, user）
                speed_j: 設置關節速度比例，範圍：1~100
                acc_j: 設置加速度比例，範圍：1~100
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f}".format(
            offset1, offset2, offset3, offset4)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovJExt(self, offset1, *dynParams):
        """
        擴展軸運動（滑軌運動）
        offset1: 位置或角度
        dynParams: 動態參數（SpeedE, AccE, Sync）
        """
        string = "MovJExt({:f}".format(
            offset1)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SyncAll(self):
        """
        等待所有指令（包含擴展軸）執行完成
        """
        string = "SyncAll()"
        return self.sendRecvMsg(string)
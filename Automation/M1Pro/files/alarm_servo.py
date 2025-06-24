alarm_servo_list = [
    {
        "id": 0,
        "level": 0,
        "en": {
            "description": "No error",
            "cause": "",
            "solution": ""
        },
        "zh_TW": {
            "description": "無錯誤",
            "cause": "",
            "solution": ""
        }
    },
    {
        "id": 25376,
        "level": 0,
        "en": {
            "description": "Abnormalities in internal servo parameters",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "伺服內部參數出現異常",
            "cause": "1.控制電源電壓瞬時下降 2.參數儲存過程中瞬時掉電 3.一定時間內參數的寫入次數超過了最大值 4.更新了軟體 5.伺服驅動器故障",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 21120,
        "level": 0,
        "en": {
            "description": "Programmable logic configuration faults",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "可程式邏輯配置故障",
            "cause": "1.FPGA和MCU軟體版本不匹配 2.FPGA故障",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29953,
        "level": 0,
        "en": {
            "description": "FPGA software version too low",
            "cause": "",
            "solution": "Please contact technical support engineer"
        },
        "zh_TW": {
            "description": "FPGA軟體版本過低",
            "cause": "",
            "solution": "請聯絡技術支援工程師"
        }
    },
    {
        "id": 29954,
        "level": 0,
        "en": {
            "description": "Programmable logic interrupt fault",
            "cause": "",
            "solution": "If connecting the power for many times, the alarm is still reported, please replace the drive"
        },
        "zh_TW": {
            "description": "可程式邏輯中斷故障",
            "cause": "1.FPGA故障 2.FPGA與MCU通訊握手異常 3.驅動器內部運算超時",
            "solution": "多次接通電源後仍報故障，更換驅動器"
        }
    },
    {
        "id": 25377,
        "level": 0,
        "en": {
            "description": "Internal program exceptions",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "內部程式異常",
            "cause": "1.EEPROM故障 2.伺服驅動器故障",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 21808,
        "level": 0,
        "en": {
            "description": "Parameter storage failure",
            "cause": "",
            "solution": "Reset the parameter and power on again, or please contact technical support engineer"
        },
        "zh_TW": {
            "description": "參數儲存故障",
            "cause": "1.參數寫入出現異常 2.參數讀取出現異常",
            "solution": "更改參數後，重新上電，或聯絡技術支援工程師"
        }
    },
    {
        "id": 28962,
        "level": 0,
        "en": {
            "description": "Product matching faults",
            "cause": "",
            "solution": "1. Check whether the motor parameter matches the motor model in nameplate; 2.Check whether the motor and driver match, otherwise, select the right motor and driver"
        },
        "zh_TW": {
            "description": "產品匹配故障",
            "cause": "1.產品編號（電機或者驅動器）不存在 2.電機與驅動器功率等級不匹配",
            "solution": "1.查看電機銘牌與電機品牌參數設定是否匹配 2.確認選擇的電機與驅動器是否配套，否則調配匹配的電機與驅動器"
        }
    },
    {
        "id": 21574,
        "level": 0,
        "en": {
            "description": "Invalid servo ON command fault",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "伺服ON指令無效故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 28964,
        "level": 0,
        "en": {
            "description": "Absolute position mode product matching fault",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "絕對位置模式產品匹配故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25378,
        "level": 0,
        "en": {
            "description": "Repeated assignment of DI functions",
            "cause": "",
            "solution": "1. Check whether the same function is assigned to different DI's; 2. Confirm whether the corresponding MCU supports the assigned functionality"
        },
        "zh_TW": {
            "description": "DI功能重複分配",
            "cause": "1.DI功能分配時，同一功能重複分配多個DI端子 2.DI功能編號超出DI功能個數 3.DI功能不支援",
            "solution": "1.檢查DI組參數，是否有同一個功能分配在不同的DI上 2.確認對應MCU版本是否支援這些分配的功能"
        }
    },
    {
        "id": 25379,
        "level": 0,
        "en": {
            "description": "DO function allocation overrun",
            "cause": "",
            "solution": "Check whether the motor and circuit are working properly, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "DO功能分配超限",
            "cause": "1.控制器異常 2.通訊線纜接觸不良或者斷開 3.通訊線纜未接地或者接地不良",
            "solution": "檢測電機和線路是否正常，或聯絡技術支援工程師"
        }
    },
    {
        "id": 29488,
        "level": 0,
        "en": {
            "description": "Data in the motor encoder ROM is incorrectly checked or parameters are not stored",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機編碼器ROM中資料校驗錯誤或未存入參數",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 8752,
        "level": 0,
        "en": {
            "description": "Hardware overcurrent",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "硬體過流",
            "cause": "1.輸入指令與接通伺服同步或輸入指令過快 2.制動電阻過小或短路 3.電機線纜接觸不良 4.電機線纜接地 5.電機UVW線纜短路 6.電機燒壞 7.增益設定不合理，電機振盪 8.編碼器接線錯誤、老化腐蝕，編碼器插頭鬆動 9.驅動器故障",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 8977,
        "level": 0,
        "en": {
            "description": "DQ axis current overflow fault",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "DQ軸電流溢出故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65288,
        "level": 0,
        "en": {
            "description": "FPGA system sampling operation timeout",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "FPGA系統採樣運算超時",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 9024,
        "level": 0,
        "en": {
            "description": "Output shorted to ground",
            "cause": "",
            "solution": "Please contact technical support engineer"
        },
        "zh_TW": {
            "description": "輸出對地短路",
            "cause": "1.驅動器動力線纜（U V W）對地發生短路 2.電機對地短路 3.驅動器故障",
            "solution": "請聯絡技術支援工程師"
        }
    },
    {
        "id": 13184,
        "level": 0,
        "en": {
            "description": "UVW phase sequence error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "UVW相序錯誤",
            "cause": "電機U V W與驅動器的U V W相序不對應",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 33922,
        "level": 0,
        "en": {
            "description": "Flying Cars",
            "cause": "",
            "solution": "Please contact technical support engineer"
        },
        "zh_TW": {
            "description": "飛車",
            "cause": "1.U V W相序接線錯誤 2.上電時，干擾信號導致電機轉子初始相位檢測錯誤 3.編碼器型號錯誤或接線錯誤 4.編碼器接線鬆動 5.負載過大",
            "solution": "請聯絡技術支援工程師"
        }
    },
    {
        "id": 12816,
        "level": 0,
        "en": {
            "description": "Electrical over-voltage in the main circuit",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "主回路電過壓",
            "cause": "1.輸入電壓等級錯誤 2.制動電阻失效 3.制動電阻過大，吸收能量速度過慢",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12832,
        "level": 0,
        "en": {
            "description": "Main circuit voltage undervoltage",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "主回路電壓欠壓",
            "cause": "1.輸入電源電壓不穩或者掉電 2.急加速過程中電壓下降明顯 3.伺服驅動器故障",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12592,
        "level": 0,
        "en": {
            "description": "Main circuit electrical shortage",
            "cause": "",
            "solution": "Check the cable connection of power, otherwise, replace the driver"
        },
        "zh_TW": {
            "description": "主回路電缺相",
            "cause": "1.輸入電源 R S T缺失2相 2.驅動器損壞",
            "solution": "1.確認輸入電源接線是否正常 2.更換驅動器"
        }
    },
    {
        "id": 12576,
        "level": 0,
        "en": {
            "description": "Control of electrical undervoltage",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "控制電欠壓",
            "cause": "1.控制電電源不穩定或者掉電 2.控制電線纜接觸不良",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 33920,
        "level": 0,
        "en": {
            "description": "Overspeed",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機超速",
            "cause": "1.U V W相序接線錯誤 2.過速度故障判定閾值設定過小 3.電機速度超調 4.驅動器損壞",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65296,
        "level": 0,
        "en": {
            "description": "Pulse output overspeed",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "脈衝輸出過速",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65282,
        "level": 0,
        "en": {
            "description": "Failure to identify angles",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "角度辨識失敗",
            "cause": "電機編碼器校零失敗",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 9040,
        "level": 0,
        "en": {
            "description": "Drive overload",
            "cause": "",
            "solution": "Replace the driver"
        },
        "zh_TW": {
            "description": "驅動器過載",
            "cause": "電機與驅動器功率不匹配",
            "solution": "更換功率匹配的驅動器"
        }
    },
    {
        "id": 29056,
        "level": 0,
        "en": {
            "description": "Motor overload",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機過載",
            "cause": "1.負載過重，實效轉矩超過額定轉矩，長時間持續運轉。2.增益調整不良導致發振、擺動動作。電機出現振動、異音。3.電機配線錯誤、斷線。4.機械受到碰撞、機械突然變重，機械扭曲。5.制動器未打開時，電機動作。6.在多台機械配線中，誤將電機線連接到其它軸，錯誤配線。",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 28961,
        "level": 0,
        "en": {
            "description": "Overheating protection for blocked motors",
            "cause": "",
            "solution": "Check whether the hardware is working properly, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機堵轉過熱保護",
            "cause": "1.電機被機械卡住 2.驅動器U V W輸出缺相或者相序錯誤",
            "solution": "檢查硬體是否正常，或聯絡技術支援工程師"
        }
    },
    {
        "id": 17168,
        "level": 0,
        "en": {
            "description": "Radiator overheating",
            "cause": "",
            "solution": "Drop the environment temperature, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "散熱器過熱",
            "cause": "1.環境溫度過高 2.驅動器風扇損壞 3.伺服驅動器內部故障",
            "solution": "降低環境溫度，或聯絡技術支援工程師"
        }
    },
    {
        "id": 29571,
        "level": 0,
        "en": {
            "description": "Encoder battery failure",
            "cause": "",
            "solution": "Connect battery, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器電池失效",
            "cause": "1.斷電期間，編碼器未接電池 2.編碼器電池電壓過低",
            "solution": "接上電池，或聯絡技術支援工程師"
        }
    },
    {
        "id": 29490,
        "level": 0,
        "en": {
            "description": "Encoder multi-turn count error",
            "cause": "",
            "solution": "Replace the motor"
        },
        "zh_TW": {
            "description": "編碼器多圈計數錯誤",
            "cause": "編碼器故障",
            "solution": "更換電機"
        }
    },
    {
        "id": 29491,
        "level": 0,
        "en": {
            "description": "Encoder multi-turn count overflow",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器多圈計數溢出",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29492,
        "level": 0,
        "en": {
            "description": "Encoder interference",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器干擾",
            "cause": "編碼器Z信號被干擾，導致Z信號對應角度變化過大",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29493,
        "level": 0,
        "en": {
            "description": "External encoder scale failure",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "外部編碼器標尺故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29494,
        "level": 0,
        "en": {
            "description": "Encoder data abnormalities",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器資料異常",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29495,
        "level": 0,
        "en": {
            "description": "Encoder return checksum exception",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器回送校驗異常",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29496,
        "level": 0,
        "en": {
            "description": "Loss of encoder Z signal",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器Z信號丟失",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 34321,
        "level": 0,
        "en": {
            "description": "Excessive position deviation",
            "cause": "",
            "solution": "Check whether the motor is working properly, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "位置偏差過大",
            "cause": "1.電機未旋轉 2.驅動器增益偏小 3.相對於運行條件，H0A.08設定過小",
            "solution": "1.確定電機是否被卡住 2.正確設定增益係數 3.設定合理的H0A.08的值"
        }
    },
    {
        "id": 34322,
        "level": 0,
        "en": {
            "description": "Position command too large",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "位置指令過大",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 34323,
        "level": 0,
        "en": {
            "description": "Excessive deviation from fully closed-loop position",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "全閉環位置偏差過大",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25380,
        "level": 0,
        "en": {
            "description": "Electronic gear setting overrun",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電子齒輪設定超限",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25381,
        "level": 0,
        "en": {
            "description": "Wrong parameter setting for fully closed loop function",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "全閉環功能參數設定錯誤",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25382,
        "level": 0,
        "en": {
            "description": "Software position upper and lower limits set incorrectly",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "軟體位置上下限設定錯誤",
            "cause": "對象字典0x607D-01h設定的數值小於0x607D-02h的值",
            "solution": "正確的設定0x607D-01h，0x607D-02h的值"
        }
    },
    {
        "id": 25383,
        "level": 0,
        "en": {
            "description": "Wrong home position offset setting",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "原點偏置設定錯誤",
            "cause": "原點偏置值在軟體位置上下限之外",
            "solution": "正確的設定0x607D-01h，0x607D-02h的值，保證原點偏置值0x607C介於二者之間"
        }
    },
    {
        "id": 30083,
        "level": 0,
        "en": {
            "description": "Loss of synchronisation",
            "cause": "",
            "solution": ""
        },
        "zh_TW": {
            "description": "同步丟失",
            "cause": "同步通訊時，主站同步信號丟失",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 30081,
        "level": 0,
        "en": {
            "description": "Unburned XML configuration file",
            "cause": "",
            "solution": "Burn the XML configuration file"
        },
        "zh_TW": {
            "description": "未燒錄XML配置檔案",
            "cause": "未燒錄設備配置檔案",
            "solution": "燒錄設備配置檔案"
        }
    },
    {
        "id": 65298,
        "level": 0,
        "en": {
            "description": "Network initialization failure",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "網路初始化失敗",
            "cause": "1.未燒錄FPGA韌體 2.未燒錄設備配置檔案 3.驅動器故障",
            "solution": "1.燒錄FPGA韌體 2.燒錄設備配置檔案 3.更換伺服驅動器"
        }
    },
    {
        "id": 30082,
        "level": 0,
        "en": {
            "description": "Sync cycle configuration error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "同步週期配置錯誤",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 30084,
        "level": 0,
        "en": {
            "description": "Excessive synchronisation period error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "同步週期誤差過大",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25384,
        "level": 0,
        "en": {
            "description": "Fault in crossover pulse output setting",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "分頻脈衝輸出設定故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65521,
        "level": 0,
        "en": {
            "description": "Zero return timeout fault",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "回零點超時故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29570,
        "level": 0,
        "en": {
            "description": "Encoder battery warning",
            "cause": "",
            "solution": "Replace battery"
        },
        "zh_TW": {
            "description": "編碼器電池警告",
            "cause": "電池電壓低於3.0V",
            "solution": "更換電池"
        }
    },
    {
        "id": 21570,
        "level": 0,
        "en": {
            "description": "DI emergency brake",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "DI緊急剎車",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12851,
        "level": 0,
        "en": {
            "description": "Motor overload warning",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機過載警告",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12817,
        "level": 0,
        "en": {
            "description": "Brake resistor overload alarm",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "制動電阻過載報警",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25385,
        "level": 0,
        "en": {
            "description": "External braking resistor too small",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "外接制動電阻過小",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 13105,
        "level": 0,
        "en": {
            "description": "Motor power cable disconnection",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電機動力線斷線",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 25386,
        "level": 0,
        "en": {
            "description": "Change of parameters requires re-powering to take effect",
            "cause": "",
            "solution": "Clear the alarm and power on again"
        },
        "zh_TW": {
            "description": "變更參數需要重新上電生效",
            "cause": "修改的參數屬於斷電生效參數",
            "solution": "清除告警，並重新上電"
        }
    },
    {
        "id": 30208,
        "level": 0,
        "en": {
            "description": "Frequent parameter storage",
            "cause": "",
            "solution": "Check whether the upper computer is working normal, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "參數儲存頻繁",
            "cause": "上位機系統反覆重新更改參數",
            "solution": "檢查上位機是否工作異常，或聯絡技術支援工程師"
        }
    },
    {
        "id": 21571,
        "level": 0,
        "en": {
            "description": "Forward overtravel warning",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "正向超程警告",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 21572,
        "level": 0,
        "en": {
            "description": "Reverse overtravel warning",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "反向超程警告",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29569,
        "level": 0,
        "en": {
            "description": "Internal failure of the encoder",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器內部故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12597,
        "level": 0,
        "en": {
            "description": "Input phase failure warning",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "輸入缺相警告",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65432,
        "level": 0,
        "en": {
            "description": "Zero return mode setting error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "回零模式設定錯誤",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 65344,
        "level": 0,
        "en": {
            "description": "Parameter recognition failure",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "參數辨識失敗",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 21121,
        "level": 0,
        "en": {
            "description": "internal error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "內部錯誤",
            "cause": "看門狗復位",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29956,
        "level": 0,
        "en": {
            "description": "FPGA configuration error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "FPGA配置錯誤",
            "cause": "FPGA初始化失敗",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 51020,
        "level": 0,
        "en": {
            "description": "Driver board identification error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "驅動板辨識錯誤",
            "cause": "PowerID錯誤",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29568,
        "level": 0,
        "en": {
            "description": "Encoder connection error",
            "cause": "",
            "solution": "Check the cable connection of encoder, or contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器連接錯誤",
            "cause": "1.編碼器插頭鬆動 2.編碼器類型設定錯誤 3.電機編碼器損壞 4.驅動器故障",
            "solution": "請檢查編碼器接線是否正常，或聯絡技術支援工程師"
        }
    },
    {
        "id": 8992,
        "level": 0,
        "en": {
            "description": "Software overcurrent",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "軟體過流",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 9088,
        "level": 0,
        "en": {
            "description": "Current zero point too large",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "電流零點過大",
            "cause": "電流採樣模組自舉失敗",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 30080,
        "level": 0,
        "en": {
            "description": "EtherCAT communication failure",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "EtherCAT通訊故障",
            "cause": "EtherCAT通訊斷開",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 33921,
        "level": 0,
        "en": {
            "description": "Excessive speed tracking error",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "速度跟蹤誤差過大",
            "cause": "1.電機堵轉 2.UVW輸出斷開 3.轉矩輸出限制 4.驅動器增益設定過小 5.驅動器或者電機損壞",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 21569,
        "level": 0,
        "en": {
            "description": "Upper and lower board connection failure",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "上下板連接故障",
            "cause": "伺服控制板與驅動板連接異常",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 8980,
        "level": 0,
        "en": {
            "description": "Busbar overcurrent",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "母線過流",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 17169,
        "level": 0,
        "en": {
            "description": "Damaged or uninstalled temperature measuring resistors",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "測溫電阻損壞或者未安裝",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 29572,
        "level": 0,
        "en": {
            "description": "Encoder Eeprom reading CRC fault",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "編碼器Eeprom讀取CRC故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    },
    {
        "id": 12928,
        "level": 0,
        "en": {
            "description": "Servo and motor power matching faults",
            "cause": "",
            "solution": "System error, please contact technical support engineer"
        },
        "zh_TW": {
            "description": "伺服和電機功率匹配故障",
            "cause": "",
            "solution": "系統錯誤，請聯絡技術支援工程師"
        }
    }
]
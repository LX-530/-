#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_APS_202304v2.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : APS
# 
#            超参数：
#            - IP: str, 表示Python_Nano_Servo4APS_[versions].py与该程序通讯的IP地址
#                   在终端使用ifconfig指令查看IP地址
#            - DEFAULT_DUTY: float, 电机默认占空比
#            - DEFAULT_POSITION: int, 舵机默认转角位置
#            - WHEEL_BASE: int or float, 小车轴距
#            - ULTRASONIC_THRESHOLD: int or float, 超声波检测阈值
#                   小于该阈值被认为存在障碍物
#            - PARKING_DIRECTION: str, "right"或"left", 表示停车位在小车右边或左边
#            - PARKING_LENGTH_THRESHOLD: int or float, 表示水平车位于垂直车位的长度阈值
#                   小于该阈值被认为是垂直停车车位；大于该阈值被认为是水平停车位
#
#            function:
#            - levelParking(): 调整水平泊车轨迹
#            - verticalParking(): 调整垂直泊车轨迹
#            - runModel(): 轨迹控制的基本模块，控制电机舵机运行直至一定时间或转过一定角度
#
# @author  : Zhonglw
# @date    : 2023/5
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
# from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
# import Python_Nano_Motor_202302V2 as Motor
# import Python_Nano_Servo_202302V2 as Servo
import Python_4S_Ultrasonic_202505V1 as Ultra
import time
import socket
import struct
import math
import fcntl
import mmap
import os

# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# from matplotlib.animation import FuncAnimation

# plt.ion()  # 开启交互模式
# fig, ax = plt.subplots(figsize=(10, 6))
# car_rect = patches.Rectangle((0, 0), 200, 420, linewidth=2, edgecolor='blue', facecolor='lightblue')
# parking_rect = None
# ax.add_patch(car_rect)
# ax.set_xlim(-500, 500)
# ax.set_ylim(-500, 500)
# ax.set_aspect('equal')
# ax.grid(True)
# plt.title("Automatic Parking Visualization")


IP = '172.20.10.5'
DEFAULT_DUTY = 0.06
DEFAULT_POSITION = 1500
WHEEL_BASE = 25     # cm
ULTRASONIC_THRESHOLD = 45   # cm
PARKING_DIRECTION = "left"     # right or left
PARKING_LENGTH_THRESHOLD = 55 # cm

Step = 1
LastData = 0


class UltraMF(Ultra.UltraObj):
    def __init__(self, filterLength: int, trigPin, echoPin):
        # 继承
        super(UltraMF, self).__init__(trigPin, echoPin, mode=Ultra.BOARD, unit="cm")
        
        self.origialData = [0.0] * int(filterLength)
        self.listLen = filterLength

    def detection(self):
        distance = Ultra.UltraObj.detection(self)
        return self._filter(distance)

    def _filter(self, data: float):
        self.origialData = self.origialData[1:]
        self.origialData.append(data)

        self.filterData = UltraMF._quicksort(self.origialData)  # 排序

        number = self.filterData[self.listLen // 2]
        if self.listLen % 2 == 1:
            return float(number)
        else:
            return float(0.5 * (number + self.filterData[self.listLen // 2 - 1]))

    @staticmethod   # 静态方法
    def _quicksort(arr: list):
        if len(arr) <= 1:
            return arr
        else:
            pivot = arr[len(arr) // 2]
            left = [x for x in arr if x < pivot]
            mid = [x for x in arr if x == pivot]
            right = [x for x in arr if x > pivot]
        return UltraMF._quicksort(left) + mid + UltraMF._quicksort(right)



# 从图像处理中获取舵机角度位置
def servoPositionSocket():
    socketLink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socketLink.bind((IP, 5000))
    return int(struct.unpack('d', socketLink.recvfrom(2048)[0])[0])


# 电机、舵机控制
def run(position=DEFAULT_POSITION, duty=DEFAULT_DUTY, stopFlag=False):
    duty = 0 if stopFlag else duty
    send_servo_position_to_server(position)
    send_data_to_server(duty)
    if stopFlag:
        time.sleep(1)
    return position, duty


# 检测停车位
def checkParking(startTime):
    global LastData
    atPosition, atVelocity = DEFAULT_POSITION, 0
    obstacleFlag, detectionFlag = False, False
    obstacleDis = []

    while True:
        position = servoPositionSocket()

        distance = RightUltra.detection() if PARKING_DIRECTION == "right" else LeftUltra.detection()

        if 5 < distance <= ULTRASONIC_THRESHOLD and not obstacleFlag:
            LastData = [distance, atVelocity, time.time()]
            detectionFlag = True

        if distance > ULTRASONIC_THRESHOLD and detectionFlag:
            obstacleDis.append(LastData)
            obstacleFlag = True

        if obstacleFlag and distance <= ULTRASONIC_THRESHOLD:
            obstacleDis.append([distance, atVelocity, time.time()])  
            break

        atPosition, atVelocity = run(position)

        if (time.time() - startTime) * atVelocity > 600:
            raise KeyboardInterrupt
        
    run(DEFAULT_POSITION, 0)
    return obstacleDis


def runModel(position=DEFAULT_POSITION, duty=DEFAULT_DUTY, **kwargs):
    """
    控制模块，控制电机舵机运行直至一定时间或转过一定角度

    :param position: 舵机角度位置，默认值DEFAULT_POSITION
    :param duty: 电机占空比，默认DEFAULT_DUTY
    :**kwargs: 运行结束条件，time或alpha
    
        time: 运行一定时间后结束，单位s
        alpha: 转过一定角度后结束，角度制
        
        Examples:
        >>> runModel(position=900, duty=0.08, time=2)
        >>> 舵机角度位置为900，电机占空比为0.08，运行2秒

        >>> runModel(duty=0.07, alpha=60)
        >>> 舵机角度位置为默认值DEFAULT_POSITION，电机占空比为0.07，小车转过60度后结束运行

    """
    global Step
    print(f"\n---------- step {Step} ----------")

    atPosition, atVelocity = run(position, duty=0, stopFlag=True)
    vel, pos = [atVelocity], [atPosition]
    startTime = time.time()

    while True:
        aveVelocity = sum(vel) / len(vel)
        aveTheta = math.radians(sum(pos) / len(pos) / 20 - 75)
        atAlpha = abs(aveVelocity) * (time.time() - startTime) * math.tan(abs(aveTheta)) / WHEEL_BASE
        
        if "alpha" in kwargs.keys():
            if atAlpha >= math.radians(kwargs["alpha"]):
                break
        elif "time" in kwargs.keys():
            if time.time() - startTime >= kwargs["time"]:
                break
        atPosition, atVelocity = run(position, duty)
        
        vel.append(atVelocity)
        pos.append(atPosition)
    run(stopFlag=True)
    Step += 1


# 垂直泊车
def verticalParking():
    if PARKING_DIRECTION == "right":
        # runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.2)
        runModel(position=850, alpha=40)
        runModel(position=2150, duty=- (DEFAULT_DUTY + 0.01), alpha=100)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.4)
        runModel(duty=0)
    
    elif PARKING_DIRECTION == "left":
        # runModel(position=2200, alpha=30)
        # runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=80)
        # runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.8)
        # runModel(duty=0)

        runModel(position=2200, time=2.5)
        runModel(position=800, duty=-DEFAULT_DUTY-0.01, time=4)
        runModel(duty=-DEFAULT_DUTY, time=0.8)
        runModel(duty=0)


# 水平泊车
def levelParking():
    if PARKING_DIRECTION == "right":
        runModel(duty=DEFAULT_DUTY + 0.01, time=0.8)
        runModel(position=2200, duty=- (DEFAULT_DUTY + 0.01), alpha=60)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.6)
        runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=30)
        runModel(position=2200, duty=DEFAULT_DUTY + 0.01, alpha=15)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.4)
        runModel(duty=0)

    if PARKING_DIRECTION == "left":
        runModel(duty=DEFAULT_DUTY + 0.01, time=1)
        runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=75)
        # runModel(duty=(DEFAULT_DUTY + 0.01), time=0.7)
        runModel(position=2100, duty=- (DEFAULT_DUTY + 0.01), alpha=53)
        runModel(position=800, duty=DEFAULT_DUTY + 0.01, alpha=15)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.3)
        runModel(duty=0)


def send_data_to_server(motor_value):
    write_data(float_val = motor_value)
    # print("电机值", motor_value)


def send_servo_position_to_server(servo_position):
    write_data(int_val= servo_position)
    # print("舵机值", servo_position)

def meminit():
    """确保共享内存文件存在且大小正确"""
    if not os.path.exists(SHM_FILE):
        # 创建文件并初始化大小
        with open(SHM_FILE, 'wb') as f:
            # 直接预分配足够空间（无需检查大小）
            f.write(struct.pack(DATA_FORMAT, INITIAL_VALUE))
            f.truncate(DATA_SIZE)  # 强制文件大小为 DATA_SIZE
        print(f"Created mmap file with size {DATA_SIZE}B")
    else:
        # 检查现有文件大小
        current_size = os.path.getsize(SHM_FILE)
        if current_size < DATA_SIZE:
            # 扩展文件大小
            with open(SHM_FILE, 'r+b') as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                f.truncate(DATA_SIZE)
                f.flush()
                print(f"Extended file size from {current_size}B to {DATA_SIZE}B")
                fcntl.flock(f, fcntl.LOCK_UN)




def write_data(int_val=None, float_val=None):
    """
    原子更新数据（允许仅更新部分字段）
    - int_val: 要更新的整型值（None 表示不更新）
    - float_val: 要更新的浮点值（None 表示不更新）
    """
    with open(SHM_FILE, 'r+b') as f:
        fcntl.flock(f, fcntl.LOCK_EX)  # 排他锁
        mm = mmap.mmap(f.fileno(), DATA_SIZE, access=mmap.ACCESS_WRITE)
        # 读取当前值
        current_data = mm.read(DATA_SIZE)
        current_int, current_float = struct.unpack(DATA_FORMAT, current_data)
        # 合并新值（保留未更新的字段）
        new_int = current_int if int_val is None else int_val
        new_float = current_float if float_val is None else float_val
        
        # 写入更新后的完整数据
        mm.seek(0)
        mm.write(struct.pack(DATA_FORMAT, new_int, new_float))
        mm.flush()  # 确保数据同步到文件
        mm.close()
        
        fcntl.flock(f, fcntl.LOCK_UN)

if __name__ == "__main__":

    SHM_FILE = "/dev/shm/servo_shared.dat"
    DATA_FORMAT = 'if'  # 格式：'i'（4字节整型） + 'f'（4字节单精度浮点） → 总8字节
    INITIAL_VALUE = 0  # 初始值
    INITIAL_SIZE = DATA_SIZE  = struct.calcsize(DATA_FORMAT)  # 初始化文件大小 

    meminit()

    RightUltra = UltraMF(5, Ultra.RIGHT_TRIG_PIN, Ultra.RIGHT_ECHO_PIN)
    LeftUltra = UltraMF(5, Ultra.LEFT_TRIG_PIN, Ultra.LEFT_ECHO_PIN)

    # send_servo_position_to_server(DEFAULT_POSITION)
    run(DEFAULT_POSITION, DEFAULT_DUTY)
    time.sleep(1)

    try:
        servoPositionSocket()   # 等待视觉程序启动

        # 检测停车位，建议使用纸箱为障碍物
        startTime = time.time()
        while True:
            obstacleDis = checkParking(startTime)

            aveDistance = sum([row[0] for row in obstacleDis]) / len(obstacleDis)   # 平均检测距离
            aveVelocity = sum([row[1] for row in obstacleDis]) / len(obstacleDis)   # 平均车速
            
            parkingLength = aveVelocity * (obstacleDis[-1][2] - obstacleDis[-2][2])
            print("parking length = ", parkingLength)
            if 0 <= parkingLength <= PARKING_LENGTH_THRESHOLD:
                print("Vertical parking space detected.")
                parkingType = 1
                break
            elif PARKING_LENGTH_THRESHOLD <= parkingLength <= 140:
                print("Horizontal parking space detected.")
                parkingType = 0
                break

            if (time.time() - startTime) * aveVelocity > 600:
                print("No parking space detected.")
                raise KeyboardInterrupt
            print("--------------------")
        
        time.sleep(1)

        if parkingType == 0:    # 水平
            levelParking()

        elif parkingType == 1:  # 垂直
            verticalParking()
        
    except KeyboardInterrupt:
        exit(0)

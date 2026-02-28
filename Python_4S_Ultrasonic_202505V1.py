#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_Ultrasonic_202305v2.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 超声波（YB-MVU04）数据读取及解析
# 
# @author  : Zhonglw
# @date    : 2023/5
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import Jetson.GPIO as GPIO
import time


# 引脚
LEFT_TRIG_PIN = 11  # 触发引脚编号
LEFT_ECHO_PIN = 13  # 信号回响引脚编号
RIGHT_TRIG_PIN = 31
RIGHT_ECHO_PIN = 33

# 编码格式
BOARD = GPIO.BOARD
BCM = GPIO.BCM


class UltraObj:
    """
    超声波对象

    __init__():
        初始化

        :param trigPin: int, 触发引脚编号

            BOARD: 7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40
            BCM: 4, 17, 18, 27, 22, 23, 24, 10, 25, 9, 11, 8, 7, 5, 6, 12, 13, 19, 16, 26, 20, 21
        
        :param echoPin: int, 信号回响引脚编号

            BOARD: 7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40
            BCM: 4, 17, 18, 27, 22, 23, 24, 10, 25, 9, 11, 8, 7, 5, 6, 12, 13, 19, 16, 26, 20, 21
        
        :param mode: 引脚编码格式，BOARD，BCM，默认为BOARD

            暂不支持CVM，TEGRA_SOC编码

        :param unit: str, 检测结果的单位，默认为米m

            mm：毫米
            cm：厘米
            m：米
    
    detection():
        检测

        :return: float, 超声波检测结果，单位为unit
    """
    BoardPin = (7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40)
    BcmPin = (4, 17, 18, 27, 22, 23, 24, 10, 25, 9, 11, 8, 7, 5, 6, 12, 13, 19, 16, 26, 20, 21)
    
    def __init__(self, trigPin, echoPin, mode=BOARD, unit="m"):
        if trigPin == echoPin:
            raise ValueError('trigPin and echoPin have the same value. They should have different values.')
        self._mode = None
        self._trigPin = None
        self._echoPin = None
        self._unit = None

        self.mode = mode
        self.trigPin = trigPin
        self.echoPin = echoPin
        self.unit = unit    # 设置检测结果的单位
        
        self.tRisingEdge = 0.0   # 上升沿的触发时间
        self.tFallingEdge = 0.0  # 下降沿的触发时间

        GPIO.setmode(self.mode)  # 使用物理编码，及开发板上正面的引脚编号
        GPIO.setup(self.trigPin, GPIO.OUT)  # 设置触发引脚为输出模式
        GPIO.setup(self.echoPin, GPIO.IN)  # 设置信号引脚为输出模式
    
    @property
    def unit(self): return self._unit

    @unit.setter
    def unit(self, unit):
        if not isinstance(unit, str):
            raise ValueError(f'"unit" must be string, not {type(unit)}.')
        if unit not in ("mm", "cm", "m"):
            raise ValueError(f'"unit" must be one of "mm", "cm", "m", not {unit}.')
        self._unit = unit

    @property
    def mode(self): return self._mode

    @mode.setter
    def mode(self, mode):
        if not isinstance(mode, int):
            raise ValueError(f'"mode", must be string, not {type(mode)}.')
        if mode not in (BOARD, BCM):
            raise ValueError(f'"mode" must be one of BOARD, BCM, not {mode}.')
        self._mode = mode

    @property
    def trigPin(self): return self._trigPin

    @trigPin.setter
    def trigPin(self, trigPin):
        if not isinstance(trigPin, int):
            raise ValueError(f'"trigPin" must be integer, not {type(trigPin)}.')
        if trigPin not in UltraObj.BoardPin and self.mode == BOARD:
            raise ValueError(f'"trigPin" muust be one of {UltraObj.BoardPin}, not {trigPin}.')
        elif trigPin not in UltraObj.BcmPin and self.mode == BCM:
            raise ValueError(f'"trigPin" muust be one of {UltraObj.BcmPin}, not {trigPin}.')
        self._trigPin = trigPin
    
    @property
    def echoPin(self): return self._echoPin

    @echoPin.setter
    def echoPin(self, echoPin):
        if not isinstance(echoPin, int):
            raise ValueError(f'"echoPin" must be integer, not {type(echoPin)}.')
        if echoPin not in UltraObj.BoardPin and self.mode == BOARD:
            raise ValueError(f'"echoPin" muust be one of {UltraObj.BoardPin}, not {echoPin}.')
        elif echoPin not in UltraObj.BcmPin and self.mode == BCM:
            raise ValueError(f'"echoPin" muust be one of {UltraObj.BcmPin}, not {echoPin}.')
        self._echoPin = echoPin

    # 输出10us的脉冲
    def _triggerPulse(self):
        GPIO.output(self.trigPin, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(self.trigPin, GPIO.LOW)

    def detection(self):
        """
        超声波测距函数，需放置在循环中执行

        :return: float, 距离，单位为unit，默认为cm
        """
        self.tRisingEdge, self.tFallingEdge = 0, 0
        
        self._triggerPulse()  # 输出10us的触发脉冲做为超声波的控制信号

        timeOutCount = 1000
        while GPIO.input(self.echoPin) == GPIO.LOW and timeOutCount > 0:
            timeOutCount -= 1
        self.tRisingEdge = time.time()

        timeOutCount = 1000
        while GPIO.input(self.echoPin) == GPIO.HIGH and timeOutCount > 0:
            timeOutCount -= 1
        self.tFallingEdge = time.time()

        # 计算距离
        distance = (self.tFallingEdge - self.tRisingEdge) * 340 / 2 * 100  # 单位cm

        if self.unit == "mm":
            return distance * 10
        
        elif self.unit == "m":
            return distance * 0.01
        
        else:
            return distance


if __name__ == "__main__":
    rUltra = UltraObj(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, unit="cm")    # 右侧超声波

    lUltra = UltraObj(LEFT_TRIG_PIN, LEFT_ECHO_PIN, unit="cm")      # 左侧超声波
    
    while True:
        print("right = ", rUltra.detection(), "\tleft = ", lUltra.detection())
        time.sleep(0.1)

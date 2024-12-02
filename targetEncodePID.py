#!/usr/bin/python

import os
import cv2
import numpy as np
import onnxruntime as ort
import RPi.GPIO as GPIO
import threading
from PCA9685 import PCA9685
import time

# 配置GPIO引脚
encoder_A = 21
encoder_B = 20

# 设置GPIO模式
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 编码器脉冲计数
pulse_count = 0
last_state = GPIO.input(encoder_A)

# 编码器回调函数
def coder_callback(channel):
    global pulse_count, last_state
    state_A = GPIO.input(encoder_A)
    state_B = GPIO.input(encoder_B)
    if state_A != last_state:
        if state_B != state_A:
            pulse_count += 1
        else:
            pulse_count -= 1
    last_state = state_A

GPIO.add_event_detect(encoder_A, GPIO.BOTH, callback=coder_callback)

# 初始化PWM控制
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver:
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, fb, speed):
        if speed < 0 or speed > 100:
            raise ValueError("Speed must be between 0 and 100")
        if motor == 0:
            pwm.setDutycycle(self.PWMA, speed)
            pwm.setLevel(self.AIN1, not fb)
            pwm.setLevel(self.AIN2, fb)
        elif motor == 1:
            pwm.setDutycycle(self.PWMB, speed)
            pwm.setLevel(self.BIN1, not fb)
            pwm.setLevel(self.BIN2, fb)
        else:
            raise ValueError("Invalid motor selection: use 0 for A, 1 for B")

    def MotorStop(self, motor):
        if motor == 0:
            pwm.setDutycycle(self.PWMA, 0)
        elif motor == 1:
            pwm.setDutycycle(self.PWMB, 0)
        else:
            raise ValueError("Invalid motor selection: use 0 for A, 1 for B")

# 创建电机驱动实例
Motor = MotorDriver()

# PID 控制类
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# 初始化PID控制器
pid_controller = PID(kp=1.0, ki=0.01, kd=0.1)

# 目标角度
target_angle = 90
pulses_per_revolution = 240
target_pulses = int((target_angle / 360) * pulses_per_revolution)

try:
    print(f"Target angle: {target_angle} degrees")
    print(f"Target pulses: {target_pulses}")

    while True:
        # 获取当前脉冲数
        current_pulses = pulse_count
        current_angle = (current_pulses / pulses_per_revolution) * 360

        # 计算PID输出
        pid_output = pid_controller.compute(target_pulses, current_pulses)

        # 将PID输出转换为速度值
        speed = min(max(abs(pid_output), 0), 100)
        direction = 1 if pid_output > 0 else 0

        # 控制电机
        Motor.MotorRun(0, direction, speed)
        Motor.MotorRun(1, direction, speed)

        # 如果接近目标值，停止电机
        if abs(target_pulses - current_pulses) < 5:
            Motor.MotorStop(0)
            Motor.MotorStop(1)
            break

        # 打印实时状态
        print(f"Current angle: {current_angle:.2f} degrees, Speed: {speed}, Direction: {'CW' if direction else 'CCW'}")
        time.sleep(0.1)

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    Motor.MotorStop(0)
    Motor.MotorStop(1)
    GPIO.cleanup()

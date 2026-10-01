import time
import sys
import math
import threading
from unitree_actuator_sdk import *


ZERO_OFFSET_DEG=56.87

serial = SerialPort('/dev/ttyUSB0')
MOTOR_ID = 0
GEAR_RATIO = queryGearRatio(MotorType.GO_M8010_6)

cmd = MotorCmd()
data = MotorData()

data.motorType = MotorType.GO_M8010_6

target_q = 0.0
running = True
lock = threading.Lock()

def deg_to_rotor_q(deg):
    return (deg + ZERO_OFFSET_DEG) * math.pi / 180 *GEAR_RATIO

def control_loop():
    global target_q, running
    current_q = data.q
    max_step = 0.003

    while running:
        with lock:
            final_q = target_q
        if current_q <final_q:
            current_q = min(current_q +max_step,final_q)
        else:
            current_q = max(current_q-max_step,final_q)
        
        cmd.motorType = MotorType.GO_M8010_6
        cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
        cmd.id   = MOTOR_ID
        cmd.q    = current_q
        cmd.dq   = 0.0
        cmd.kp   = 1.0
        cmd.kd   = 0.05
        cmd.tau  = 0.0

        serial.sendRecv(cmd,data)
        time.sleep(0.001)
        

cmd.motorType = MotorType.GO_M8010_6
cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
cmd.id   = MOTOR_ID
cmd.q    = 0.0
cmd.dq   = 0.0
cmd.kp   = 0.0
cmd.kd   = 0.0
cmd.tau  = 0.0
serial.sendRecv(cmd,data)
time.sleep(0.1)

with lock:
    target_q = data.q

t = threading.Thread(target=control_loop,daemon=True)
t.start()

print("正在回归0位置")
with lock:
    target_q = deg_to_rotor_q(0.0)
time.sleep(5.0)

print("已就绪,输入输出轴角度,输入q退出")

while True:
    s = input("角度（度）：").strip()
    if s.lower() == 'q':
        break
    try:
        deg = float(s)
    except ValueError:
        print("请输入数字：")
        continue
    
    if abs(deg)>360:
        print("建议输入-360~360度")
        continue
    with lock:
        target_q = deg_to_rotor_q(deg)
    print(f"目标输出角度：{deg},转子侧 q={target_q:.4f} rad")

running = False
time.sleep(0.1)
print("退出")

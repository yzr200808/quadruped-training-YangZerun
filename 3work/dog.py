import time
import mujoco
import mujoco.viewer
import numpy as np
import xml.etree.ElementTree as ET

model = mujoco.MjModel.from_xml_path("black_description.xml")
data = mujoco.MjData(model)
current_key = '1'


motors = ET.parse("black_description.xml").getroot().findall("./actuator/motor")
joint_names = [motor.get("joint") for motor in motors]
num_motors = len(joint_names)
q_adrs = np.zeros(num_motors, dtype=int)
dq_adrs = np.zeros(num_motors, dtype=int)
act_ids = np.zeros(num_motors, dtype=int)

pose_1 = np.zeros(num_motors)
pose_1[1]=1.2
pose_1[2] = -2.5
pose_1[4] = -1.2
pose_1[5] = 2.5
pose_1[7] = -1.2
pose_1[8] = 2.5
pose_1[10] = 1.2
pose_1[11] = -2.5
pose_2 = np.zeros(num_motors)
pose_2[1]= 0
pose_2[2] = -0.85
pose_2[4] = 0
pose_2[5] = 0.85
pose_2[7] = -0.6
pose_2[8] = 1.1
pose_2[10] = 0.6
pose_2[11] = -1.1
pose_dict = {'1':pose_1, '2':pose_2}
def key_callback(keycode):
    global current_key
    if chr(keycode) == '9':
        current_key = '1'
    elif chr(keycode) == '8':
        current_key = '2'

for i, motor in enumerate(motors):
    # 获取每个电机的关节ID和执行器ID
    jnt_id =mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, motor.get("joint"))
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor.get("name"))
    # 将关节ID和执行器ID对应在data.qpos和data.qvel中的位置储存在数组中
    q_adrs[i] = model.jnt_qposadr[jnt_id]
    dq_adrs[i] = model.jnt_dofadr[jnt_id]
    act_ids[i] = act_id

data.qpos[q_adrs] = pose_1
data.qvel[:]=0
mujoco.mj_forward(model, data)

q_des = np.array(data.qpos[q_adrs])
dq_des = np.zeros(num_motors)

last_key = current_key
t_start = time.time()
q_start = q_des.copy()
T = 3.0

kp = np.array([80,80,80,80,80,80,80,350,350,80,350,350])
kd = np.array([3,3,3,3,3,3,3,3,3,3,3,3])
tau_max = np.full(num_motors, 33.5)


with mujoco.viewer.launch_passive(model, data ,key_callback=key_callback) as viewer:

    while viewer.is_running():
        step_start = time.time()

        if current_key != last_key:
            t_start = time.time()
            q_start = q_des.copy()
            last_key = current_key
                
        
        target_q_des = pose_dict[current_key]
        t = time.time() - t_start
        if t >= T:
            q_des = target_q_des.copy()
        else:
            tau_norm = t/T
            s = 10*tau_norm**3 - 15*tau_norm**4 + 6*tau_norm**5
            q_des = q_start + (target_q_des - q_start) * s
        #读取当前状态
        q = data.qpos[q_adrs]
        dq = data.qvel[dq_adrs]
        #获取重力补偿力矩
        tau_gravity = data.qfrc_bias[dq_adrs]
        
        
        #PD+重力补偿
        tau = kp * (q_des - q) + kd *(dq_des - dq) + tau_gravity
        #限幅
        tau = np.clip(tau, -tau_max, tau_max)
        #写入执行器
        data.ctrl[act_ids] = tau
        #推进物理
        mujoco.mj_step(model, data)
        #刷新画面
        viewer.sync()
        #实时同步
        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)
# MIT 控制参数（控制器节点 -> 仿真/驱动节点）
#
# 数组下标顺序固定为 black_description.xml 中 actuator 的顺序：
#   0-2 : FL hip / thigh / calf
#   3-5 : FR hip / thigh / calf
#   6-8 : RR hip / thigh / calf
#   9-11: RL hip / thigh / calf
#
# 单位：q [rad]，dq [rad/s]，tau [Nm]
# tau 是控制器按公式算好的 PD 力矩（不含重力补偿）：
#   tau = kp * (q - q_now) + kd * (dq - dq_now)
# 下游写入执行器的力矩为 tau 再叠加自己负责的重力补偿（限幅由模型 ctrlrange 完成）：
#   tau_cmd = tau + qfrc_bias

std_msgs/Header header

float64[12] kp      # 位置刚度
float64[12] kd      # 速度阻尼
float64[12] q       # 期望关节角
float64[12] dq      # 期望关节角速度
float64[12] tau     # PD 力矩（不含重力补偿），重力补偿由下游节点自行叠加

# 仿真/驱动节点回传的状态（仿真节点 -> 控制器节点）
#
# 数组下标顺序与 MitCmd 一致（FL / FR / RR / RL，每个腿 hip / thigh / calf）
# 单位：q [rad]，dq [rad/s]，ddq [rad/s^2]，tau [Nm]，cur [A]

std_msgs/Header header

float64[12] q     # 当前关节角
float64[12] dq    # 当前关节角速度
float64[12] ddq   # 当前关节角加速度（MuJoCo 中即 data.qacc）
float64[12] tau   # 实际施加的关节力矩（已按 ctrlrange 限幅，MuJoCo 中即 data.actuator_force）
float64[12] cur   # 电机电流（仿真侧目前为占位全 0，接真机时用于回传实测电流）

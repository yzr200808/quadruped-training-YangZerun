// 四足 MIT 参数控制器节点
//
// 按窗口按键（经 /key_input 转发）决定发送哪一组 12 电机 MIT 参数
// {kp, kd, q, dq, tau} 到 /mit_cmd，由仿真节点消费并回传 /mit_state：
//   - 按键 8：站姿保持。kp/kd 取参数，q 从当前实测关节角五次多项式插值到
//             站姿，dq 恒为 0；tau 按公式算出 PD 力矩（不含重力补偿），
//             重力补偿由仿真节点自行叠加。
//   - 按键 9：松弛。kp/q/dq 为 0，kd 取 limp_kd（默认 0.01），tau 只剩阻尼力矩；
//             仿真节点识别到 kp 全 0 后不再叠加重力补偿，电机实际被放开。
// 开机即处于松弛模式；旧按键 9 对应的“趴下固定姿态”已删除，趴下姿态只作为
// 仿真节点的初始关节角存在。
//
// 与 dog.py 的关系：站姿保持这一路与原控制律等价
//   tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_gravity

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mit_interfaces/msg/mit_cmd.hpp"
#include "mit_interfaces/msg/mit_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{
//constexpr:编译阶段已经算好的常量     size_t:无符号整型
constexpr std::size_t kNumMotors = 12;
//按键映射
constexpr int kKeyStand = 8;  ///< 8 = 站姿保持
constexpr int kKeyLimp = 9;   ///< 9 = 松弛（电机放开）

//多项式插值函数
double quinticBlend(double u)
{
  u = std::clamp(u, 0.0, 1.0);    //clamp：确保u在0-1范围内再返回
  const double u3 = u * u * u;   
  return 10.0 * u3 - 15.0 * u3 * u + 6.0 * u3 * u * u;
}

std::array<double, kNumMotors> toMotorArray(
  const std::vector<double> & values, const std::string & name)
{
  if (values.size() != kNumMotors) {
    throw std::runtime_error(
            "参数 " + name + " 需要 " + std::to_string(kNumMotors) + " 个元素，实际为 " +
            std::to_string(values.size()));
  }
  std::array<double, kNumMotors> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

} 
//把ros2中的参数数组转成固定长度的数组，并校验其长度为12

class MitControllerNode : public rclcpp::Node
{
public:
  MitControllerNode()
  : rclcpp::Node("mit_controller_node")
  {
    kp_ = toMotorArray(
      declare_parameter<std::vector<double>>(
        "kp", {80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 350.0, 350.0, 80.0, 350.0, 350.0}),
      "kp");
    kd_ = toMotorArray(
      declare_parameter<std::vector<double>>(
        "kd", std::vector<double>(kNumMotors, 3.0)),
      "kd");
    stand_pose_ = toMotorArray(
      declare_parameter<std::vector<double>>(
        "stand_pose",
        {0.0, 0.3, -0.85, 0.0, -0.3, 0.85, 0.0, -0.6, 1.1, 0.0, 0.6, -1.1}),
      "stand_pose");
    //读取三组电机参数：kp、kd、stand_pose

    blend_time_ = declare_parameter<double>("blend_time", 3.0);//创建参数，尖括号内是数据类型，小括号内第一个是
    publish_rate_ = declare_parameter<double>("publish_rate", 500.0);//参数名，第二个是参数值
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.1);
    limp_kd_ = declare_parameter<double>("limp_kd", 0.01);
    //读取四个标量参数：站姿插值时间，发布频率，反馈超时阈值，松弛阻尼系数

    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/mit_cmd");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/mit_state");
    const auto key_topic = declare_parameter<std::string>("key_topic", "/key_input");
    const bool cmd_best_effort = declare_parameter<bool>("cmd_best_effort", true);
    const bool state_best_effort = declare_parameter<bool>("state_best_effort", true);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");

    if (blend_time_ < 0.0) {
      throw std::runtime_error("参数 blend_time 不能为负");
    }
    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("参数 publish_rate 必须为正");
    }
    if (limp_kd_ < 0.0) {
      throw std::runtime_error("参数 limp_kd 不能为负");
    }
    //参数检查

    //初始状态：开机即松弛，不发任何目标位置
    mode_ = Mode::kLimp;
    q_start_ = {};
    target_q_ = stand_pose_;
    q_cmd_ = {};
    blend_start_time_ = now();

    //接口设置
    rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));//只保留最新一条命令
    cmd_qos.durability_volatile();//新订阅者不会收到旧命令
    if (cmd_best_effort) {
      cmd_qos.best_effort();
    } else {
      cmd_qos.reliable();
    }//根据参数选择 QoS 策略

    // 创建发布者                 消息类型                       话题名      QoS配置
    cmd_pub_ = create_publisher<mit_interfaces::msg::MitCmd>(cmd_topic, cmd_qos);

    // 状态反馈同样只保留最新一条；两端 QoS 必须一致，否则收不到数据
    rclcpp::QoS state_qos(rclcpp::KeepLast(1));
    state_qos.durability_volatile();
    if (state_best_effort) {
      state_qos.best_effort();
    } else {
      state_qos.reliable();
    }
    state_sub_ = create_subscription<mit_interfaces::msg::MitState>(
      state_topic, state_qos,
      [this](const mit_interfaces::msg::MitState &msg){onState(msg);});

    key_sub_ = create_subscription<std_msgs::msg::Int32>(
      key_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const std_msgs::msg::Int32 &msg){onKeyInput(msg);});

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::Imu &msg){onImu(msg);});
    //创建四个订阅者

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    //以秒为单位，用double表示的持续时间（时间间隔）
    control_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
    //转化为纳秒，存入control_period_

    RCLCPP_INFO(
      get_logger(),
      "MIT 控制器已启动：发布 %s（%.1f Hz，%s），订阅 %s（%s），按键话题 %s（%d=站姿，%d=松弛），"
      "插值时长 %.2f s，limp_kd %.3f，初始模式 松弛",
      cmd_topic.c_str(), publish_rate_, cmd_best_effort ? "best-effort" : "reliable",
      state_topic.c_str(), state_best_effort ? "best-effort" : "reliable",
      key_topic.c_str(), kKeyStand, kKeyLimp,
      blend_time_, limp_kd_);
    //打印启动信息

    startControlThread();//启动控制线程
  }

  ~MitControllerNode() override
  {
    running_ = false;                //通知控制线程退出
    if (control_thread_.joinable()) {
      control_thread_.join();        //等待控制线程结束，回收资源
    }
  }

private:
  void startControlThread()
  {
    control_thread_ = std::thread(
      [this]() {
        auto next = std::chrono::steady_clock::now() + control_period_;
        //初始化下一次唤醒时间
        while (running_ && rclcpp::ok()) {
          std::this_thread::sleep_until(next);//睡到目标时刻
          onTimer();                           //执行控制任务
          next += control_period_;             //累加周期
          const auto now_steady = std::chrono::steady_clock::now();
          if (now_steady - next > control_period_) {
            next = now_steady + control_period_;  // 落后检测，严重落后时重新对齐，不补跑积压
          }
        }
      });
  }

  void onState(const mit_interfaces::msg::MitState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);  //加锁保护共享数据
    for (std::size_t i = 0; i < kNumMotors; ++i) {
      q_fb_[i] = msg.q[i];    //缓存最新关节位置
      dq_fb_[i] = msg.dq[i];  //缓存最新关节速度
    }
    last_feedback_time_ = now();    //记录最后一次收到反馈的时间
    has_feedback_ = true;           //标记“是否已经收到过反馈”

    // 若站姿指令早于首帧反馈到达，这里才开始插值：以实测位置为起点，避免从 0 起跳
    if (hold_pending_) {
      hold_pending_ = false;
      q_start_ = q_fb_;
      target_q_ = stand_pose_;
      blend_start_time_ = now();
      RCLCPP_INFO(get_logger(), "已收到首帧状态反馈，开始从当前位姿插值到站姿");
    }
  }

  /// IMU 数据（目前只缓存并按 1 Hz 打印，供后续姿态控制使用）
  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      imu_quat_ = {
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w};
      imu_gyro_ = {
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
      imu_accel_ = {
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
    }//缓存imu数据
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU 姿态(xyzw)=[%.3f %.3f %.3f %.3f] 角速度=[%.3f %.3f %.3f] rad/s "
      "加速度=[%.3f %.3f %.3f] m/s^2",
      imu_quat_[0], imu_quat_[1], imu_quat_[2], imu_quat_[3],
      imu_gyro_[0], imu_gyro_[1], imu_gyro_[2],
      imu_accel_[0], imu_accel_[1], imu_accel_[2]);
      //每秒打印一次imu数据
  }

  /// 窗口按键：控制器按内置按键表决定发哪一组 MIT 参数
  void onKeyInput(const std_msgs::msg::Int32 & msg)
  {
    switch (msg.data) {
      case kKeyStand:
        enterStand();
        break;
      case kKeyLimp:
        enterLimp();
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "忽略按键 %d（当前只支持 %d=站姿，%d=松弛）",
          msg.data, kKeyStand, kKeyLimp);
        break;
    }
  }

  /// 进入站姿保持：从当前实测关节角平滑接管到站姿
  void enterStand()
  {
    bool deferred = false;//记录本次调用是否需要延迟到首帧反馈到达后再开始插值
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mode_ = Mode::kStand;
      if (!has_feedback_) {
        // 还没有实测位置：先挂起，等首帧反馈到达后再开始插值
        hold_pending_ = true;
        deferred = true;
      } else {
        hold_pending_ = false;
        q_start_ = q_fb_;
        target_q_ = stand_pose_;
        blend_start_time_ = now();
      }
    }
    if (deferred) {
      RCLCPP_INFO(get_logger(), "收到站姿指令，等待首帧状态反馈后再开始插值");
    } else {
      RCLCPP_INFO(
        get_logger(), "进入站姿保持：%.2f s 内从当前位姿插值到站姿", blend_time_);
    }
  }

  /// 进入松弛：kp/q/dq/tau 全 0，只保留微小阻尼；仿真节点据此不再叠加重力补偿
  void enterLimp()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mode_ = Mode::kLimp;
      hold_pending_ = false;
      q_cmd_ = {};
    }
    RCLCPP_INFO(
      get_logger(), "进入松弛：kp=0，kd=%.3f，q/dq/tau=0（电机放开）", limp_kd_);
  }

  /// 定时器回调：按当前模式计算 MIT 命令并发布
  void onTimer()
  {
    const rclcpp::Time stamp = now();

    mit_interfaces::msg::MitCmd cmd;
    //创建命令

    cmd.header.stamp = stamp;
    cmd.header.frame_id = "base";
    //给命令打上时间戳和参考坐标系

    bool not_fresh = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      not_fresh = !has_feedback_ ||
        (stamp - last_feedback_time_).seconds() > feedback_timeout_;

      if (not_fresh) {
        // 没有可用反馈：发全 0，仿真侧按“kp 全 0”识别为松弛，等同于放开电机
        cmd.kp = {};
        cmd.kd = {};
        cmd.q = {};
        cmd.dq = {};
        cmd.tau = {};
      } else if (mode_ == Mode::kLimp) {
        // 松弛：不给任何目标位置，tau 只剩阻尼力矩 kd*(0-dq)，不含重力补偿
        cmd.kp = {};
        cmd.kd.fill(limp_kd_);
        cmd.q = {};
        cmd.dq = {};
        for (std::size_t i = 0; i < kNumMotors; ++i) {
          cmd.tau[i] = limp_kd_ * (0.0 - dq_fb_[i]);
        }
      } else {
        // 站姿保持：按公式算出 PD 力矩（不含重力补偿）填进 tau，
        // 重力补偿由仿真节点自行叠加
        updateCommand(stamp);
        cmd.kp = kp_;
        cmd.kd = kd_;
        cmd.q = q_cmd_;
        cmd.dq = {};  // dog.py 中 dq_des 恒为 0
        for (std::size_t i = 0; i < kNumMotors; ++i) {
          cmd.tau[i] = kp_[i] * (q_cmd_[i] - q_fb_[i]) + kd_[i] * (0.0 - dq_fb_[i]);
        }
      }
    }

    if (not_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "超过 %.0f ms 未收到状态反馈，输出松弛命令（kp=kd=tau=0）",
        feedback_timeout_ * 1000.0);
    }

    cmd_pub_->publish(cmd);
  }

  /// 计算本时刻的期望关节角（五次多项式插值），结果写入 q_cmd_
  void updateCommand(const rclcpp::Time & stamp)
  {
    const double elapsed = (stamp - blend_start_time_).seconds();
    const double s = (blend_time_ <= 0.0 || elapsed >= blend_time_)
      ? 1.0 : quinticBlend(elapsed / blend_time_);

    for (std::size_t i = 0; i < kNumMotors; ++i) {
      q_cmd_[i] = q_start_[i] + (target_q_[i] - q_start_[i]) * s;
    }
  }

  // 参数
  std::array<double, kNumMotors> kp_{};
  std::array<double, kNumMotors> kd_{};
  std::array<double, kNumMotors> stand_pose_{};   ///< 站姿目标（旧 pose_2）
  std::array<double, kNumMotors> target_q_{};     ///< 当前目标姿态
  std::array<double, kNumMotors> q_start_{};      ///< 插值起点
  std::array<double, kNumMotors> q_cmd_{};        ///< 最近一次发布的位置指令

  /// 控制模式：松弛（电机放开）或站姿保持
  enum class Mode { kLimp, kStand };
  Mode mode_{Mode::kLimp};
  bool hold_pending_{false};  ///< 站姿指令早于首帧反馈时挂起，待反馈后起算插值

  double blend_time_{3.0};
  double publish_rate_{500.0};
  double feedback_timeout_{0.1};
  double limp_kd_{0.01};

  // 最新反馈缓存（q/dq 目前只做记录，供后续扩展与调试使用）
  std::mutex mutex_;
  std::array<double, kNumMotors> q_fb_{};
  std::array<double, kNumMotors> dq_fb_{};

  // 最新 IMU 数据：姿态四元数 (x,y,z,w)、角速度、线加速度
  std::array<double, 4> imu_quat_{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> imu_gyro_{};
  std::array<double, 3> imu_accel_{};

  bool has_feedback_{false};
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time blend_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<mit_interfaces::msg::MitCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mit_interfaces::msg::MitState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr key_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::chrono::nanoseconds control_period_{2000000};
  std::atomic<bool> running_{true};
  std::thread control_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);//初始化ros2
  try {
    rclcpp::spin(std::make_shared<MitControllerNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mit_controller_node"), "节点启动失败：%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026 z1_ros2_ws contributors
//
// Z1 机械臂的直接力矩控制:
//
//   tau = clamp(tau_cmd, -effort_limit, +effort_limit)
//
// 这是力矩控制栈的最底层:把 `Float64MultiArray` 话题里的原始关节力矩
// 原样转发到各关节的 effort 命令接口。刻意**不含**位置伺服、轨迹逻辑
// 和重力补偿——重力补偿、导纳控制等上层算法作为发布者接在它上面,
// 就像真机上 LOWCMD 状态是 z1_ctrl 的原始力矩入口一样。
//
// 两道安全机制防止出错的发布者把机械臂卡死:
//
//   * `effort_limits.<joint>` 在每个力矩写出前做限幅
//     (默认值与 URDF 的 effort 限幅一致:30 N·m,joint2 为 60 N·m)。
//   * `command_timeout` 是看门狗:超过该时长没有新指令到达,
//     输出被强制清零。设为 0.0 则关闭看门狗、保持最后一条指令。
//
// 实时性纪律(与 Z1JointTrajectoryController 相同):
//
//   * 订阅回调只做校验、把消息拷贝进 realtime_tools::RealtimeBuffer;
//     update() 从缓冲区读取。载荷是 shared_ptr,所以实时循环不产生分配。
//   * 所有暂存缓冲区都在 on_init() 里一次性定尺寸,绝不在 update() 里分配。

#ifndef Z1_CONTROLLERS__Z1_TORQUE_CONTROLLER_HPP_
#define Z1_CONTROLLERS__Z1_TORQUE_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace z1_controllers
{

class Z1TorqueController : public controller_interface::ControllerInterface
{
public:
  Z1TorqueController() = default;

  // ---- 生命周期(各回调的细节见 .cpp) ------------------------------------
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 订阅回调:校验消息(元素个数必须等于 dof_)后交给缓冲区,由实时循环
  /// 取用。运行在控制器(非实时)执行器线程上,绝不直接碰硬件状态。
  void on_torque_command(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

  // ---- 配置(on_init 里一次性读取) ----------------------------------------
  /// 受控关节;同时定义了入站 `data` 数组的元素顺序。
  std::vector<std::string> joints_;
  /// 每关节的力矩限幅,按惯例 = 该关节的 URDF effort 限幅。
  std::vector<double> effort_limits_;
  /// 看门狗周期,单位秒(0.0 = 关闭,保持最后一条指令)。
  double command_timeout_{0.25};

  // ---- 运行时状态 ----------------------------------------------------------
  /// 自由度数 = joints_.size()。
  size_t dof_{0};
  /// update() 的暂存缓冲区,on_init() 里一次性定尺寸,实时循环零分配。
  /// 保存最近一条被接受的指令(首条指令到来前或看门狗超时后为零)。
  std::vector<double> tau_;
  /// 订阅回调到 update() 之间的实时安全交接通道。
  realtime_tools::RealtimeBuffer<std::shared_ptr<const std::vector<double>>> torque_buffer_;
  /// 最近一次被 update() 采纳的指令 shared_ptr。缓冲区会永远持有最后
  /// 一条消息,用指针同一性判断"是否是新消息"(每条消息都是新建的
  /// shared_ptr),否则看门狗会被旧消息每周期刷新、永远不会触发。
  std::shared_ptr<const std::vector<double>> last_seen_command_;
  /// 话题 `~/torque_commands` 的订阅(解析为
  /// `/<控制器名>/torque_commands`)。显式 `.reliable()`(KeepLast(1)):
  /// 不依赖中间件默认(SystemDefaultsQoS 在 FastDDS 上解析成
  /// BEST_EFFORT,会与默认发布端静默断连),且与 `ros2 topic pub`
  /// 默认 QoS 一致。
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
  /// 最近一条被接受指令的 update() 时刻——看门狗的时钟。用 update()
  /// 的 `time` 而不是回调里的墙钟,使看门狗不受执行器调度抖动影响。
  rclcpp::Time last_command_time_;
  /// 第一条指令被接受前为 false;此期间输出保持为零,因此仅仅激活
  /// 控制器绝不会让机械臂动起来。
  bool have_command_{false};
  /// update() 发现命令接口拒写时置位;只上报一次随后清除
  /// (与 Z1JointTrajectoryController 同样的模式)。
  std::atomic<bool> command_write_failed_{false};
  /// 超时告警已打印过时为 true,下一条新指令到来时清除,避免高速
  /// 实时循环每个周期刷一条告警。
  std::atomic<bool> timeout_reported_{false};
};

}  // namespace z1_controllers

#endif  // Z1_CONTROLLERS__Z1_TORQUE_CONTROLLER_HPP_

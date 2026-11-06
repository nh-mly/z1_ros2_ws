// Copyright 2026 z1_ros2_ws contributors
//
// 这个控制器做什么(原始力矩转发,无伺服律、无重力补偿)、以及本文件
// 遵循的实时性规则,见 include/z1_controllers/z1_torque_controller.hpp。

#include "z1_controllers/z1_torque_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace z1_controllers
{

// ------------------------------------------------------------------ on_init
//
// 读取本控制器认识的全部参数。所有默认值都定义在这里,因此只给一个
// `joints` 列表控制器就能工作;YAML 只覆盖它想覆盖的部分。参数仅在
// 不存在时才声明,因为 ros2_control 禁止重复声明同名参数(controller_manager
// 可能不重建节点就重新 configure)。

controller_interface::CallbackReturn Z1TorqueController::on_init()
{
  auto node = get_node();
  const auto logger = node->get_logger();

  // 驱动哪些关节、以什么顺序。必填:没有它就没有可 claim 的接口,
  // 也没有给 effort_limits 建立索引的依据。
  if (!node->has_parameter("joints"))
  {
    node->declare_parameter("joints", std::vector<std::string>{});
  }
  joints_ = node->get_parameter("joints").get_value<std::vector<std::string>>();
  if (joints_.empty())
  {
    RCLCPP_ERROR(logger, "Parameter 'joints' must list the controlled joints.");
    return controller_interface::CallbackReturn::ERROR;
  }
  dof_ = joints_.size();

  // 输出暂存缓冲区:在这里一次性定尺寸,之后绝不再重分配。
  tau_.assign(dof_, 0.0);

  // 每关节一个力矩限幅。30 N·m 默认值是机械臂通用 URDF effort 限幅
  // (const.xacro 里的 torqueMax);joint2 是双电机关节,在 YAML 里给 60 N·m。
  // 限幅在 update() 里执行,因此即使指令大得离谱也到不了硬件。
  effort_limits_.resize(dof_, 30.0);
  for (size_t i = 0; i < dof_; ++i)
  {
    const std::string limit_name = "effort_limits." + joints_[i];
    if (!node->has_parameter(limit_name))
    {
      node->declare_parameter(limit_name, effort_limits_[i]);
    }
    effort_limits_[i] = node->get_parameter(limit_name).get_value<double>();
  }

  // 看门狗。负数没有意义,直接夹到 0(= 关闭看门狗)。
  if (!node->has_parameter("command_timeout"))
  {
    node->declare_parameter("command_timeout", command_timeout_);
  }
  command_timeout_ = std::max(0.0, node->get_parameter("command_timeout").get_value<double>());

  RCLCPP_INFO(
    logger, "Z1 torque controller: %zu joints, timeout %.3f s (0 = hold last command).", dof_,
    command_timeout_);
  return controller_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------- 接口 claim
//
// 每关节 claim 一个 effort 命令接口,并 claim position + velocity 状态
// 接口。状态接口当前只用于占位(与 Z1JointTrajectoryController 一致)——
// 教训:旧版 ros2_control 里"纯命令、零状态"的控制器在 gz_ros2_control
// 下 update() 不生效,力矩写不出去;claim 状态接口后一切正常。将来做
// 重力补偿时这两个状态接口直接可用。

controller_interface::InterfaceConfiguration
Z1TorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration Z1TorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

// --------------------------------------------------------------- on_configure
//
// 创建订阅。这是非实时的生命周期迁移,允许自由分配内存。缓冲区先用
// 空载荷占位,保证 update() 从第一个周期起就有东西可读。

controller_interface::CallbackReturn Z1TorqueController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // `~/` 相对控制器自己的节点名解析,即完整话题是
  // `/<控制器名>/torque_commands`。显式 `.reliable()`:命令流不允许
  // 静默丢包,且与 `ros2 topic pub` 的默认 QoS 一致、开箱即连。
  // (教训:SensorDataQoS 是 BEST_EFFORT;SystemDefaultsQoS 在 FastDDS
  // 上同样解析成 BEST_EFFORT——两者与默认发布的 RELIABLE 不兼容,
  // 会静默断连、`Publisher count: 0`。)
  torque_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/torque_commands", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&Z1TorqueController::on_torque_command, this, std::placeholders::_1));
  torque_buffer_.writeFromNonRT(std::shared_ptr<const std::vector<double>>());
  return controller_interface::CallbackReturn::SUCCESS;
}

// --------------------------------------------------------------- on_activate
//
// 干净地进入力矩模式:输出从零开始,激活前发布过的任何指令都被丢弃,
// 这样切换控制器时绝不会把上一次会话的陈旧力矩施加到机械臂上。
// 命令接口也显式清零,即使上一个控制器是崩溃退出而非正常去激活,
// 硬件也处于已知状态。

controller_interface::CallbackReturn Z1TorqueController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(tau_.begin(), tau_.end(), 0.0);
  have_command_ = false;
  timeout_reported_ = false;
  last_seen_command_.reset();
  torque_buffer_.writeFromNonRT(std::shared_ptr<const std::vector<double>>());

  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(0.0))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Cannot write command interface '%s'.",
        command_interfaces_[i].get_name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "Z1 torque controller active; publishing to '~/torque_commands'.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------- on_deactivate
//
// 不再控制硬件的那一刻就停止向硬件灌力矩(否则 Gazebo 会在没有控制器
// 拥有该接口期间继续对最后那个力矩积分)。缓冲的指令一并丢弃,
// 保证重新激活时又是干净状态。

controller_interface::CallbackReturn Z1TorqueController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(0.0))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Cannot reset command interface '%s'.",
        command_interfaces_[i].get_name().c_str());
      break;
    }
  }
  have_command_ = false;
  last_seen_command_.reset();
  torque_buffer_.writeFromNonRT(std::shared_ptr<const std::vector<double>>());
  return controller_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------- update
//
// 在实时循环上以 controller_manager 的 update_rate(本工作区为 250 Hz)
// 运行。每个周期:
//
//   1. 若订阅回调留下了新指令,取用最新一条;
//   2. 运行看门狗(发布者失联则输出清零);
//   3. 把每个力矩夹到限幅内并写出。
//
// 这里没有任何分配:tau_ 是成员变量,缓冲区交接的是 shared_ptr,
// 唯一的堆操作只是缓冲区内部的引用计数。

controller_interface::return_type Z1TorqueController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // (1) 自上个周期以来到达的最新指令。`*readFromRT()` 返回内部存储的
  // 指针,解引用后得到 shared_ptr 引用;`const auto` 拷贝它只是引用计数,
  // 不拷贝载荷、不分配。
  //
  // 关键:缓冲区永远持有最后一条消息,必须用指针同一性判断"新消息"
  // (每条消息都是回调里新建的 shared_ptr),否则每个周期都会把旧消息
  // 当新指令刷新 last_command_time_,看门狗永远不会触发。
  const auto command = *torque_buffer_.readFromRT();
  if (command && command != last_seen_command_)
  {
    for (size_t i = 0; i < dof_ && i < command->size(); ++i)
    {
      tau_[i] = command->at(i);
    }
    last_seen_command_ = command;
    // 看门狗以 update() 自己的时间为时钟:相对控制循环是确定的,
    // 不受回调在哪个执行器线程上被调度的影响。
    last_command_time_ = time;
    have_command_ = true;
    timeout_reported_ = false;   // 为下一次失联重新武装告警
  }

  // (2) 看门狗。理由:它下面没有位置伺服,卡死的力矩会让关节永远转
  // 下去——输出清零把"发布者死了"变成"机械臂软掉",这在仿真里(以及
  // 将来在真机上)是可预测、可恢复的失效模式。
  if (
    command_timeout_ > 0.0 && have_command_ &&
    (time - last_command_time_).seconds() > command_timeout_)
  {
    std::fill(tau_.begin(), tau_.end(), 0.0);
    // 每次失联只打一条告警而不是每周期一条:exchange() 读取并清除。
    if (!timeout_reported_.exchange(true))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "No torque command for %.3f s; forcing the output to zero.",
        command_timeout_);
    }
  }

  // (3) 限幅并写出。非有限值(如出错发布者发来的 NaN)变成零,
  // 而不是毒化仿真。
  bool ok = true;
  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!std::isfinite(tau_[i]))
    {
      tau_[i] = 0.0;
    }
    const double clamped = std::clamp(tau_[i], -effort_limits_[i], effort_limits_[i]);
    if (!command_interfaces_[i].set_value(clamped))
    {
      command_write_failed_ = true;   // 每轮故障只上报一次,见下
      ok = false;
    }
  }

  // 写失败每轮只上报一次;exchange() 顺带清除锁存。
  if (command_write_failed_.exchange(false))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to write an effort command interface; is the hardware still active?");
  }

  return ok ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

// --------------------------------------------------------- 订阅回调
//
// 运行在控制器的执行器线程上(非实时):只做校验和暂存。长度不匹配的
// 消息丢弃并打节流告警——错误长度的消息几乎总是发布者的 bug,
// 而按错误顺序错位驱动关节比完全不动更糟。

void Z1TorqueController::on_torque_command(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != dof_)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Torque command has %zu entries, expected %zu (order: %s); dropping it.",
      msg->data.size(), dof_, joints_.front().c_str());
    return;
  }
  // 拷贝成 shared_ptr 载荷并跨过实时边界交接;update() 下个周期取用。
  // 从本线程出发,writeFromNonRT 是唯一合法的生产侧入口。
  torque_buffer_.writeFromNonRT(std::make_shared<const std::vector<double>>(msg->data));
}

}  // namespace z1_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  z1_controllers::Z1TorqueController, controller_interface::ControllerInterface)

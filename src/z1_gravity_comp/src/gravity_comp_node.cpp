// Copyright 2026 z1_ros2_ws contributors
//
// Z1 无爪机械臂的重力补偿节点(编号 2)。
//
// 在编号 1(Z1TorqueController,力矩底座)之上工作:
//
//   /joint_states ──▶ 本节点 ──KDL RNE──▶ τ_grav = G(q)
//        └──(发布)──▶ /z1_torque_controller/torque_commands ──▶ 编号 1 ──▶ 仿真
//
// 两种模式:
//   zero_g     τ = gravity_factor * τ_grav(q)
//              臂悬浮,可手拖(零力示教);松手即停。
//   soft_hold  τ = τ_grav(q) + Kp*(q_hold - q) - Kd*qdot
//              弱 PD 保持:拖得动,松手缓缓回到使能时的位姿。
//
// 安全:
//   * gravity_factor(0..1)支持从 0 缓升,激活过程无阶跃;
//   * 输出按 effort_limits 限幅(默认 30 N·m,joint2 = 60,与 URDF 一致);
//   * /joint_states 超过 joint_timeout 未更新则停止发布——编号 1 的
//     看门狗(0.25 s)会把输出清零,臂自然回落(可预测的失效模式)。
//
// QoS(踩坑记录,见 步骤2 文档):
//   * 订阅 /joint_states 用 BEST_EFFORT(joint_state_broadcaster 是
//     SensorDataQoS 发布,RELIABLE 会静默断连);
//   * 发布 torque_commands 用显式 RELIABLE(编号 1 的订阅端是
//     QoS(KeepLast(1)).reliable())。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "kdl/chaindynparam.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
constexpr int kDefaultRateHz = 250;
constexpr double kDefaultGravity = 9.81;
constexpr double kDefaultHoldP = 10.0;
constexpr double kDefaultHoldD = 5.0;
}  // namespace

class GravityCompNode : public rclcpp::Node
{
public:
  GravityCompNode()
  : Node("gravity_comp")
  {
    // ---- 参数(带默认值,launch 只覆盖需要的) ----
    base_link_ = declare_parameter("base_link", std::string("link00"));
    tip_link_ = declare_parameter("tip_link", std::string("link06"));
    rate_ = std::max(1.0, declare_parameter("rate", double(kDefaultRateHz)));
    gravity_factor_ = std::clamp(
      declare_parameter("gravity_factor", 0.0), 0.0, 1.0);
    joint_timeout_ = std::max(
      0.1, declare_parameter("joint_timeout", 1.0));
    mode_ = declare_parameter("mode", std::string("zero_g"));
    if (mode_ != "zero_g" && mode_ != "soft_hold") {
      RCLCPP_WARN(get_logger(), "unknown mode '%s', falling back to zero_g", mode_.c_str());
      mode_ = "zero_g";
    }

    // 运行时参数更新:否则 ros2 param set 只改参数服务器,成员变量永远
    // 是启动时的快照(重力系数缓升会形同虚设——实测教训)。
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        for (const auto & p : params) {
          if (p.get_name() == "gravity_factor") {
            gravity_factor_ = std::clamp(p.as_double(), 0.0, 1.0);
            RCLCPP_INFO(get_logger(), "gravity_factor -> %.2f", gravity_factor_);
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // ---- 从 robot_description 参数建立 KDL 链(与仿真同源的 URDF) ----
    // 注意:必须先 declare 再读,否则 launch 注入的参数读不到。
    std::string urdf = declare_parameter("robot_description", std::string(""));
    if (urdf.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "parameter 'robot_description' is empty; launch this node with the "
        "same xacro output as robot_state_publisher");
      throw std::runtime_error("missing robot_description");
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree)) {
      throw std::runtime_error("failed to parse robot_description into a KDL tree");
    }
    if (!tree.getChain(base_link_, tip_link_, chain_)) {
      throw std::runtime_error(
        "no chain from '" + base_link_ + "' to '" + tip_link_ + "' in the URDF");
    }
    const size_t n = chain_.getNrOfJoints();
    if (n == 0) {
      throw std::runtime_error("KDL chain has zero joints");
    }
    chain_dyn_ = std::make_unique<KDL::ChainDynParam>(
      chain_, KDL::Vector(0.0, 0.0, -kDefaultGravity));

    // 关节名 → 链内索引(链顺序 = URDF 关节顺序 = joint1..joint6)
    for (size_t i = 0; i < n; ++i) {
      const std::string name = chain_.getSegment(i).getJoint().getName();
      name_to_idx_[name] = i;
      RCLCPP_INFO(get_logger(), "chain joint[%zu] = %s", i, name.c_str());
    }

    // 每关节的限幅与保持增益(链序)
    effort_limits_.assign(n, 30.0);
    hold_p_.assign(n, kDefaultHoldP);
    hold_d_.assign(n, kDefaultHoldD);
    for (size_t i = 0; i < n; ++i) {
      const std::string name = chain_.getSegment(i).getJoint().getName();
      const double default_limit = (name == "joint2") ? 60.0 : 30.0;
      effort_limits_[i] = declare_parameter(
        "effort_limits." + name, default_limit);
      hold_p_[i] = declare_parameter("hold_gains." + name + ".p", kDefaultHoldP);
      hold_d_[i] = declare_parameter("hold_gains." + name + ".d", kDefaultHoldD);
    }

    // 运行时缓冲(链序,一次性定尺寸)
    q_.assign(n, 0.0);
    qdot_.assign(n, 0.0);
    hold_.assign(n, 0.0);
    kdl_q_.resize(n);
    kdl_qdot_.resize(n);
    kdl_tau_.resize(n);

    // ---- 话题与服务 ----
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1).best_effort(),
      std::bind(&GravityCompNode::on_joint_state, this, _1));
    torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/z1_torque_controller/torque_commands", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable", std::bind(&GravityCompNode::on_enable, this, _1, _2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&GravityCompNode::on_timer, this));

    // 超时判定基准 = 构造时刻:启动瞬间(第一帧还没到)不打伪警告。
    last_joint_state_ = now();

    RCLCPP_INFO(
      get_logger(),
      "GravityCompNode ready: %zu joints, chain %s -> %s, mode %s, %.0f Hz. "
      "Publishing to /z1_torque_controller/torque_commands.",
      n, base_link_.c_str(), tip_link_.c_str(), mode_.c_str(), rate_);
  }

private:
  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    // 按 name 索引,不假设消息里的关节顺序
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = name_to_idx_.find(msg->name[i]);
      if (it == name_to_idx_.end()) {
        continue;
      }
      const size_t idx = it->second;
      if (i < msg->position.size()) {
        q_[idx] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        qdot_[idx] = msg->velocity[i];
      }
    }
    last_joint_state_ = now();
    received_state_ = true;
    // soft_hold 使能时若还没有过状态(enable 早于首帧),补锁存保持点——
    // 否则 hold_=0 会把臂往零位猛拽。
    if (hold_pending_) {
      hold_ = q_;
      hold_pending_ = false;
      RCLCPP_INFO(get_logger(), "soft_hold hold pose latched (deferred)");
    }
  }

  void on_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    enable_ = request->data;
    if (enable_ && mode_ == "soft_hold") {
      // 只在有新鲜状态时锁存当前位姿;否则推迟到首帧到达(on_joint_state)。
      if (received_state_) {
        hold_ = q_;
        RCLCPP_INFO(get_logger(), "soft_hold enabled, holding current pose");
      } else {
        hold_pending_ = true;
        RCLCPP_INFO(get_logger(), "soft_hold enabled; hold pose deferred to first joint state");
      }
    }
    response->success = true;
    response->message = enable_ ? "gravity compensation enabled"
                                : "gravity compensation disabled";
  }

  void on_timer()
  {
    // 状态过期:停止发布(编号 1 的看门狗会清零输出,臂自然回落)
    if ((now() - last_joint_state_).seconds() > joint_timeout_) {
      if (!stale_reported_.exchange(true)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no /joint_states for %.1f s; not publishing", joint_timeout_);
      }
      return;
    }
    stale_reported_ = false;

    if (!enable_) {
      return;   // 未使能:不发布(由编号 1 看门狗兜底)
    }

    // RNE:τ_grav = G(q)
    for (size_t i = 0; i < q_.size(); ++i) {
      kdl_q_(i) = q_[i];
      kdl_qdot_(i) = qdot_[i];
    }
    chain_dyn_->JntToGravity(kdl_q_, kdl_tau_);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(q_.size());
    for (size_t i = 0; i < q_.size(); ++i) {
      double tau = gravity_factor_ * kdl_tau_(i);
      if (mode_ == "soft_hold") {
        tau += hold_p_[i] * (hold_[i] - q_[i]) - hold_d_[i] * qdot_[i];
      }
      tau = std::clamp(tau, -effort_limits_[i], effort_limits_[i]);
      msg.data[i] = tau;
    }
    torque_pub_->publish(msg);
  }

  // ---- 配置 ----
  std::string base_link_;
  std::string tip_link_;
  double rate_;
  double gravity_factor_;
  double joint_timeout_;
  std::string mode_;

  // ---- KDL ----
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> chain_dyn_;
  std::map<std::string, size_t> name_to_idx_;

  // ---- 运行时状态(链序) ----
  std::vector<double> q_;
  std::vector<double> qdot_;
  std::vector<double> hold_;
  std::vector<double> effort_limits_;
  std::vector<double> hold_p_;
  std::vector<double> hold_d_;
  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_qdot_;
  KDL::JntArray kdl_tau_;
  rclcpp::Time last_joint_state_;
  bool enable_{false};
  bool received_state_{false};
  bool hold_pending_{false};
  std::atomic<bool> stale_reported_{false};
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ---- 通信 ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GravityCompNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("gravity_comp"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}

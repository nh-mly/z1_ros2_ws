#!/usr/bin/env python3
"""
方案 A 复位:重力补偿之后让机械臂回到官方起始位姿(startFlat)。

流程:
  1. 关闭重力补偿(/gravity_comp/enable,服务不存在则跳过);
  2. 请求 z1_ctrl 的 BACKTOSTART——机械臂**自动**缓速运动到
     savedArmStates.csv 的 startFlat(≈[0, 0, -0.005, -0.074, 0, 0]);
  3. 用 /joint_states 观测到位(位置接近 startFlat 且速度≈0),报告结果。

注意:
  * 执行期间请有人轻扶监护,不要与臂较劲;
  * BACKTOSTART 期间 FSM 会在 BACKTOSTART 与 JOINTCTRL 之间跳,
    因此不用 FSM 状态判断到位,而用关节位形收敛判断;
  * 从 LOWCMD 出发,BACKTOSTART 是合法出口(State_LowCmd 出口掩码
    0x4006 含 bit14);TOSTATE(去 forward)从 LOWCMD 不可达。

用法:
  ros2 run z1_ros2_control reset_to_start.py
  ros2 run z1_ros2_control reset_to_start.py --no-gc        # GC 未在跑时
  ros2 run z1_ros2_control reset_to_start.py --timeout 30   # 自定义超时(秒)
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from z1_ros2_control.srv import GetFSMState, SetFSMState

START_FLAT = [0.0, 0.0, -0.005, -0.074, 0.0, 0.0]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
POS_TOL = 0.08      # 到位位置容差 rad
VEL_TOL = 0.08      # 到位速度容差 rad/s
SETTLE_SECS = 1.0   # 连续满足容差多久算到位


def main():
    parser = argparse.ArgumentParser(description="Z1 重力补偿后的在位复位(方案 A)")
    parser.add_argument("--no-gc", action="store_true", help="跳过关闭重力补偿")
    parser.add_argument("--timeout", type=float, default=30.0, help="等待到位的最长秒数")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("reset_to_start")

    q = {}
    qd = {}

    def cb(msg):
        for i, n in enumerate(msg.name):
            q[n] = msg.position[i]
            if i < len(msg.velocity):
                qd[n] = msg.velocity[i]

    node.create_subscription(JointState, "/joint_states", cb, JS_QOS)

    def spin_for(secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(node, timeout_sec=0.02)

    # ---- 1. 关闭重力补偿 ----
    if not args.no_gc:
        cli = node.create_client(SetBool, "/gravity_comp/enable")
        if cli.wait_for_service(timeout_sec=2.0):
            fut = cli.call_async(SetBool.Request(data=False))
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                rclpy.spin_once(node, timeout_sec=0.05)
            print("[1/3] 重力补偿已关闭")
        else:
            print("[1/3] /gravity_comp/enable 不存在,跳过(GC 未在运行)")
    else:
        print("[1/3] --no-gc:跳过关闭重力补偿")

    # ---- 2. 请求 BACKTOSTART ----
    fsm = node.create_client(SetFSMState, "/z1/set_fsm_state")
    if not fsm.wait_for_service(timeout_sec=5.0):
        print("[错误] /z1/set_fsm_state 服务不可用——ROS 链路是否在跑?")
        return
    req = SetFSMState.Request()
    req.state = "BACKTOSTART"
    req.label = ""
    fut = fsm.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    resp = fut.result() if fut.done() else None
    if resp is None or not resp.success:
        msg = resp.message if resp else "无响应"
        print(f"[2/3] BACKTOSTART 请求失败: {msg}")
        return
    print("[2/3] BACKTOSTART 已请求:机械臂自动缓速运动到 startFlat,请监护 ...")

    # ---- 3. 用位形收敛判断到位 ----
    t0 = time.time()
    settled_since = None
    while time.time() - t0 < args.timeout:
        spin_for(0.1)
        if all(n in q for n in JOINTS) and all(n in qd for n in JOINTS):
            pos_ok = all(abs(q[n] - START_FLAT[i]) < POS_TOL for i, n in enumerate(JOINTS))
            vel_ok = all(abs(qd[n]) < VEL_TOL for n in JOINTS)
            if pos_ok and vel_ok:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since > SETTLE_SECS:
                    qq = [round(q[n], 4) for n in JOINTS]
                    print(f"[3/3] 复位完成:q = {qq} (startFlat ≈ {START_FLAT})")
                    return
            else:
                settled_since = None

    qq = [round(q.get(n, float('nan')), 4) for n in JOINTS]
    print(f"[3/3] 超时未确认到位,当前 q = {qq};请用 get_fsm_state 与目视确认")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

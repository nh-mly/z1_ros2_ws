#!/usr/bin/env python3
"""GC 空中悬浮终验:PD 驱动到 forward 位形 → GC 接住(高空)→ 关 GC 观察坠落。

forward 位形 q=[0, 1.5, -1.0, -0.54, 0, 0],腕部 z≈0.40 m——纯空中,无任何接触。
注意:q2 正向 = 臂向下(0°=折叠朝上,π=下垂),"坠落"= q2 增大。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
Q_DES = [0.0, 1.5, -1.0, -0.54, 0.0, 0.0]
KP, KD = 300.0, 5.0
LIMITS = [30.0, 60.0, 30.0, 30.0, 30.0, 30.0]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def main():
    rclpy.init()
    node = rclpy.create_node("gc_forward_test")
    pub = node.create_publisher(
        Float64MultiArray, "/z1_torque_controller/torque_commands", 10)
    q = {}
    qd = {}

    def cb(msg):
        for i, n in enumerate(msg.name):
            q[n] = msg.position[i]
            if i < len(msg.velocity):
                qd[n] = msg.velocity[i]

    node.create_subscription(JointState, "/joint_states", cb, JS_QOS)
    cli = node.create_client(SetBool, "/gravity_comp/enable")

    def set_enable(v, timeout=3.0):
        fut = cli.call_async(SetBool.Request(data=v))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.done()

    def getq():
        return [q.get(n, 0.0) for n in JOINTS]

    if not cli.wait_for_service(timeout_sec=5.0):
        print("[error] enable 服务不可用", flush=True)
        return

    t0 = time.time()
    while "joint2" not in q and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f"起始 q = {[round(x, 3) for x in getq()]}", flush=True)

    # ---- 阶段 1:PD 闭环驱动到 forward(GC 关闭,避免双发布者互串)----
    set_enable(False)
    t0 = time.time()
    while time.time() - t0 < 12.0:
        qq = getq()
        err = max(abs(qq[i] - Q_DES[i]) for i in range(6))
        if err < 0.05:
            break
        m = Float64MultiArray()
        m.data = [max(-LIMITS[i], min(LIMITS[i],
                  KP * (Q_DES[i] - qq[i]) - KD * qd.get(JOINTS[i], 0.0)))
                  for i in range(6)]
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.002)
        time.sleep(0.004)
    print(f"PD 到达 q = {[round(x, 3) for x in getq()]}", flush=True)

    # ---- 阶段 2:立即停发 + GC 接住(抢在坠落前)----
    set_enable(True)
    time.sleep(2.0)
    rclpy.spin_once(node, timeout_sec=0.05)
    q_held = [round(x, 3) for x in getq()]
    print(f"GC 接住 2s: q = {q_held}", flush=True)

    # ---- 阶段 3:关 GC,观察坠落 ----
    set_enable(False)
    for k in range(5):
        time.sleep(1.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        qq = [round(x, 3) for x in getq()]
        print(f"关 GC t+{k + 1}s: q = {qq}  q2={qq[1]:.3f}", flush=True)

    q_end = getq()
    drop = abs(q_held[1] - q_end[1])
    print(f"判定:q2 从 {q_held[1]} 变到 {q_end[1]:.3f}(变化 {drop:.3f} rad)",
          "= GC 在空中位形生效 ✅" if drop > 0.3 else "(变化不足,检查)",
          flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

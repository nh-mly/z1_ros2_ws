#!/usr/bin/env python3
"""示教准备:PD 驱动到 forward 空中位形 → 使能 GC 并持续保持。Ctrl-C 退出。

退出后 GC 节点仍处于使能状态(服务端标志不随脚本退出),臂继续悬浮;
停止悬浮:ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: false}"
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

rclpy.init()
node = rclpy.create_node("gc_hold")
pub = node.create_publisher(Float64MultiArray, "/z1_torque_controller/torque_commands", 10)
q, qd = {}, {}

def cb(msg):
    for i, n in enumerate(msg.name):
        q[n] = msg.position[i]
        if i < len(msg.velocity):
            qd[n] = msg.velocity[i]

node.create_subscription(JointState, "/joint_states", cb, JS_QOS)
cli = node.create_client(SetBool, "/gravity_comp/enable")
while not cli.wait_for_service(timeout_sec=5.0):
    pass

def set_enable(v):
    fut = cli.call_async(SetBool.Request(data=v))
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)

t0 = time.time()
while "joint2" not in q and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.05)

# PD 驱动到 forward(GC 关)
set_enable(False)
t0 = time.time()
while time.time() - t0 < 12.0:
    qq = [q.get(n, 0.0) for n in JOINTS]
    if max(abs(qq[i] - Q_DES[i]) for i in range(6)) < 0.05:
        break
    m = Float64MultiArray()
    m.data = [max(-LIMITS[i], min(LIMITS[i],
              KP * (Q_DES[i] - qq[i]) - KD * qd.get(JOINTS[i], 0.0)))
              for i in range(6)]
    pub.publish(m)
    rclpy.spin_once(node, timeout_sec=0.002)
    time.sleep(0.004)

# GC 接住并持续保持
set_enable(True)
print("已到达 forward,GC 保持中。去 GUI 拖吧!(Ctrl-C 结束)", flush=True)
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()

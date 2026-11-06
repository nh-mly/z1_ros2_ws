# z1_gravity_comp

Z1 无爪机械臂的**重力补偿节点(编号 2)**:用 KDL 递归牛顿-欧拉从 URDF 动力学
算出重力矩 `τ_grav = G(q)`,发布到编号 1(`Z1TorqueController`)的
`/z1_torque_controller/torque_commands` 话题,让机械臂获得**零重力(悬浮)**
行为,用于零力示教。

## 数据流

```
/joint_states ──(BEST_EFFORT 订阅)──▶ 本节点 ──KDL RNE──▶ τ_grav
        └──(RELIABLE 发布)──▶ /z1_torque_controller/torque_commands ──▶ 编号 1 ──▶ 仿真
```

本节点不写 effort 接口、不碰 ros2_control,只是编号 1 之上的一层发布者;
真机 LOWCMD 阶段可直接复用同一节点。

## 模式

| 模式 | 输出 | 效果 |
|---|---|---|
| `zero_g`(默认) | `τ = gravity_factor · τ_grav(q)` | 臂悬浮,推动即走、松手即停 |
| `soft_hold` | `τ = τ_grav + Kp·(q_hold − q) − Kd·q̇` | 弱 PD 保持:拖得动,松手缓缓回到使能位姿 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_description` | —(launch 注入) | 与 robot_state_publisher 同源的 URDF 字符串 |
| `base_link` / `tip_link` | link00 / link06 | KDL 链端点(无爪) |
| `rate` | 250 Hz | 计算/发布频率 |
| `gravity_factor` | 0.0 | 重力系数 0..1,建议从 0 缓升到 1 |
| `mode` | zero_g | `zero_g` / `soft_hold` |
| `joint_timeout` | 1.0 s | `/joint_states` 超时则停止发布(编号 1 看门狗兜底清零) |
| `effort_limits.<joint>` | 30(joint2=60) | 输出限幅,与 URDF 一致 |
| `hold_gains.<joint>.{p,d}` | 10 / 5 | soft_hold 增益(远小于 JTC 的 300/5) |

## 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `~/enable` | `std_srvs/srv/SetBool` | 运行时开关补偿;soft_hold 使能时锁存当前位姿 |

## 用法

```bash
# 1. 力矩模式仿真(编号 1)先行
ros2 launch z1_bringup gazebo_ros2_control.launch.py use_gripper:=false \
  use_torque_control:=true \
  controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers_gz_torque.yaml

# 2. 启动重力补偿节点(默认 gravity_factor=0,不输出)
ros2 launch z1_gravity_comp gravity_comp.launch.py

# 3. 使能并缓升重力系数
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: true}"
ros2 param set /gravity_comp gravity_factor 1.0

# 4. 验证:任意位形下臂悬浮;GUI 里拖连杆随动、松手停住
#    关闭:enable false 或 gravity_factor 0.0
```

## 验证判定

| 现象 | 结论 |
|---|---|
| 停发抬臂指令后臂悬住(不塌地) | ✅ 重力补偿生效 |
| GUI 手拖连杆,臂随动、松手停 | ✅ 零力示教 |
| `/joint_states` effort ≈ 节点算出的 τ_grav | ✅ 数值正确 |
| enable=false 后臂立刻回落 | ✅ 开关有效 |

## 已知特性

* URDF 关节摩擦 1 N·m 让"零重力"手感略黏(拖拽有阻力),仿真固有。
* 本节点停止/未使能时,编号 1 的看门狗(0.25 s)自动清零输出,臂回落——
  这是可预测的失效模式,不是缺陷。

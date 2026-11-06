# z1_ros2_ws — Unitree Z1 的 ROS 2（Jazzy）工作空间

Unitree Z1 机械臂的 ROS 2 移植：URDF、Gazebo Sim、ros2_control 和 MoveIt 2 均已跑通，
真机（经官方 `z1_controller`）也已跑通：激活保持、单关节点动、`BACKTOSTART`、FSM 服务、
陈旧参考位姿保护都已实测。

参数、接口、设计原因和已知限制都写在**各包自己的 README** 里，本文件只做入口。
真机的细节（依赖、启动顺序、实机实测数据、还没验证的部分）在
[`z1_ros2_control/README.md`](src/z1_ros2_control/README.md)。

## 功能包

| 功能包 | 状态 | 内容 |
|---|---|---|
| [`z1_description`](src/z1_description) | ✅ | URDF/xacro、网格模型、RViz 配置、`display.launch.py` |
| [`z1_bringup`](src/z1_bringup) | ✅ | ros2_control / Gazebo 启动、控制器配置 |
| [`z1_moveit_config`](src/z1_moveit_config) | ✅ | SRDF、运动学、关节限位、MoveIt 2 演示 |
| [`z1_controllers`](src/z1_controllers) | ✅ | 官方力矩律控制器 `Z1JointTrajectoryController`（τ = Kp·e + Kd·ė，限幅），给 Gazebo 的 effort 接口用 |
| [`z1_ros2_control`](src/z1_ros2_control) | ✅ | 真机硬件接口（官方 `z1_controller` 的 UDP 客户端）+ FSM 服务（`~/set_fsm_state`、`~/get_fsm_state`） |

## 编译

```bash
cd ~/Projects/z1_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

依赖均已安装；可选 `ros-jazzy-trac-ik-kinematics-plugin`（比 KDL 更好的 IK）。

## 运行

| 用途 | 命令 |
|---|---|
| 仅 URDF（RViz + 关节滑块） | `ros2 launch z1_description display.launch.py` |
| ros2_control，真机（`z1_ros2_control/Z1System`） | 先在 `~/Projects/z1_controller/build` 起 `./z1_ctrl`，再 `ros2 launch z1_bringup control.launch.py` |
| Gazebo Sim（gz 位置伺服） | `ros2 launch z1_bringup gazebo.launch.py` |
| Gazebo Sim + ros2_control | `ros2 launch z1_bringup gazebo_ros2_control.launch.py` |
| MoveIt 2 演示（仿真 + RViz） | `ros2 launch z1_moveit_config demo.launch.py` |
| MoveIt 2 演示（真机 + RViz） | `ros2 launch z1_moveit_config demo.launch.py use_gazebo:=false` |

常用参数：`use_gripper:=true|false`、`use_rviz:=true|false`、`headless:=true`（Gazebo）、
`world:=<file>`、`gz_verbosity:=0..4`。

**本机机械臂没有末端执行器时**，所有涉及模型的命令都加 `use_gripper:=false`——它会把
URDF、SRDF 和控制器映射一起切到 6 轴无夹爪版本（两种配置的默认值都是 `true`）。
Gazebo + ros2_control 默认按**官方力矩律**驱动机械臂（[`z1_controllers`](src/z1_controllers)
走 effort 接口，增益 300/5，限幅 = URDF effort）；真机路径仍是位置接口 +
`joint_trajectory_controller`。要回到位置伺服：
`controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers.yaml`。

快速验证：

```bash
# 轨迹（真机 / gz_ros2_control 两条路径均可）
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.5, -0.4, 0.3, 0.2, 0.1],
              time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# 夹爪；0 = 闭合，负值 = 张开
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: -0.8, max_effort: 10.0}}"

# 单关节，仅 Gazebo gz-servo 路径可用
ros2 topic pub --once /joint1/cmd_pos std_msgs/msg/Float64 "{data: 0.6}"
```

## 实机力矩控制与重力补偿(命令说明)

> 纯命令速查见 [`实机验证步骤.md`](实机验证步骤.md);本节说明每条命令的用途、
> 预期结果与注意事项。前置:`z1_ctrl` 已编译并运行、本机网卡 `192.168.123.x`、
> **全程有人扶臂**。所有终端先
> `source /opt/ros/jazzy/setup.bash && source ~/z1_ros2_ws/install/setup.bash`。

### 阶段 B:最小真机验证

#### B1. JOINTCTRL 基准

```bash
source /opt/ros/jazzy/setup.bash && source ~/z1_ros2_ws/install/setup.bash
ros2 launch z1_bringup control.launch.py use_gripper:=false
```

* 用途:用默认配置验证位置后端(JOINTCTRL)正常。
* 预期:`ros2 control list_controllers` 显示 JSB + JTC active;臂被伺服律持住;
  `get_fsm_state` 返回 `JOINTCTRL`。
* 退出:Ctrl-C(臂软塌,先扶住)。

#### B2. LOWCMD 零力矩

```bash
ros2 launch z1_bringup control.launch.py use_gripper:=false \
  use_torque_control:=true \
  controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers_torque.yaml
```

```bash
ros2 service call /z1/get_fsm_state z1_ros2_control/srv/GetFSMState "{}"
```

* 用途:启动力矩后端;组件激活力矩控制器时会自动把 z1_ctrl 切到 LOWCMD。
* 关键知识:z1_ctrl 的 FSM **只允许 PASSIVE→LOWCMD**(反汇编
  `State_JointSpace::checkChange` 掩码 `0x4ffa` 不含 bit16,`State_Passive`
  掩码 `0x1500c` 含 bit16),组件自动走两步。
* 日志判读:`command mode switch -> LOWCMD` = 成功;
  `FSM transition to LOWCMD failed` = 被拒(臂留在 JOINTCTRL,安全,立即停止)。
* `get_fsm_state` 预期:`state='LOWCMD'`,扶臂者感到臂变软。

#### B3. 小力矩微动

```bash
timeout 6 ros2 topic pub --rate 250 /z1_torque_controller/torque_commands \
  std_msgs/msg/Float64MultiArray "{data: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```bash
ros2 topic echo --qos-reliability best_effort /joint_states --field effort
```

* 用途:验证力矩到达真机电机(只动 joint1——铅垂轴,无重力负载,最安全)。
* 前置:launch 必须在运行(否则 `/joint_states` 不发布,echo 报
  `does not appear to be published yet`)。
* 判读:effort(joint1)≈10 = 力矩到电机;≈0 = z1_ctrl 打包层清零(继续查
  IOUDP);中间值 = 钳位/损耗。
* 异常:剧烈动作/异响 → 立即 Ctrl-C launch(客户端停发 → z1_ctrl 自动 PASSIVE)。

#### B4. 安全退出

```bash
ros2 service call /z1/set_fsm_state z1_ros2_control/srv/SetFSMState "{state: PASSIVE, label: ''}"
# 然后 Ctrl-C launch
```

* 预期:`PASSIVE acknowledged`;臂完全软塌(扶住)。

### 阶段 C:真机完整重力补偿

#### C1. GC 节点

```bash
ros2 launch z1_gravity_comp gravity_comp.launch.py
ros2 param set /gravity_comp gravity_factor 1.0
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: true}"
```

* 用途:先起 GC 并备好 τ_grav(顺序关键:GC 必须先于机械臂)。
* 预期:T2 日志出现 `gravity_factor -> 1.00`;此时臂未启动,`no /joint_states`
  警告正常;节点保持运行。

#### C2. 力矩模式

```bash
ros2 launch z1_bringup control.launch.py use_gripper:=false \
  use_torque_control:=true \
  controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers_torque.yaml
```

* 预期序列:硬件激活 → JOINTCTRL 持住 → 力矩控制器激活 → LOWCMD →
  GC 的 τ_grav 接管,臂悬浮("轻若无物")。

#### C3. 悬浮/示教验证(无命令)

* 轻推臂 → 轻盈随动;松手 → 停住;漂移/下坠 → 查 `gravity_factor` 日志。

#### C4. 关闭对照

```bash
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: false}"
```

* 预期:臂立刻变沉/下坠(扶住)——证明悬浮是 GC 的功劳。

#### C5. 退出(无命令)

* `set_fsm_state PASSIVE` → Ctrl-C 两个 launch → 停 `z1_ctrl`。

### 完整流程(从开机到复位)

#### T0. z1_ctrl

```bash
cd ~/z1_controller/build && ./z1_ctrl
```

* 用途:官方底层控制器(UDP 8071/8072,loopback);一启动臂就 PASSIVE,先扶住。

#### T2. GC 节点

```bash
source /opt/ros/jazzy/setup.bash && source ~/z1_ros2_ws/install/setup.bash
ros2 launch z1_gravity_comp gravity_comp.launch.py
```

* 用途:先起 GC;`no /joint_states` 警告正常,节点别关。

#### T3. 使能 GC

```bash
ros2 param set /gravity_comp gravity_factor 1.0
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: true}"
# 回 T2 确认日志出现 "gravity_factor -> 1.00"
```

* 用途:重力系数 1.0 + 使能;必须确认日志,否则 GC 输出恒 0。

#### T1. 力矩模式

```bash
source /opt/ros/jazzy/setup.bash && source ~/z1_ros2_ws/install/setup.bash
ros2 launch z1_bringup control.launch.py use_gripper:=false \
  use_torque_control:=true \
  controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers_torque.yaml
```

* 预期:JOINTCTRL → LOWCMD → GC 接管,臂变轻。

#### 验证

```bash
ros2 control list_controllers
ros2 service call /z1/get_fsm_state z1_ros2_control/srv/GetFSMState "{}"   # LOWCMD
ros2 topic echo --qos-reliability best_effort /joint_states --field effort --once
# joint2/3/4 应有重力支撑力矩(几 N·m)
```

* 判读:JSB + 力矩控制器 active、`state='LOWCMD'`、joint2/3/4 有重力支撑力矩;
  手感推/停。

#### 关闭对照

```bash
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: false}"
```

* 预期:臂立刻变沉(下坠,扶住)。

#### 复位

```bash
ros2 run z1_ros2_control reset_to_start.py
```

* 预期:臂自动缓速回 `startFlat` 并停住。

#### 结束

```bash
ros2 service call /z1/get_fsm_state z1_ros2_control/srv/GetFSMState "{}"
# 然后 Ctrl-C 关 T2/T1,最后停 z1_ctrl
```

### 步骤 5:复位(方案 A)

#### 一键脚本

```bash
ros2 run z1_ros2_control reset_to_start.py
# 可选参数:
#   --no-gc      GC 节点没在跑时跳过"关闭重力补偿"
#   --timeout N  等待到位的最长秒数(默认 30)
```

* 流程:关 GC → 请求 `BACKTOSTART` → 用关节位形收敛判断到位。
* 为什么不用 FSM 判到位:BACKTOSTART 期间 FSM 在 BACKTOSTART/JOINTCTRL 间跳。
* 依据:反汇编 `State_LowCmd` 出口掩码 `0x4006` 含 bit14(BACKTOSTART 合法);
  `TOSTATE`(去 forward)从 LOWCMD 不可达。
* 运动形式:机械臂**自动**缓速运动,人只做轻扶监护。

#### 手动命令

```bash
ros2 service call /gravity_comp/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /z1/set_fsm_state z1_ros2_control/srv/SetFSMState "{state: BACKTOSTART, label: ''}"
ros2 service call /z1/get_fsm_state z1_ros2_control/srv/GetFSMState "{}"
```

### 关键知识(反汇编 `libZ1_x86_64.so` 所得)

#### FSM 转移规则

| 转移 | 合法? | 证据 |
|---|---|---|
| JOINTCTRL → LOWCMD | ❌ | `State_JointSpace::checkChange` 掩码 `0x4ffa` 无 bit16 |
| PASSIVE → LOWCMD | ✅ 唯一入口 | `State_Passive` 掩码 `0x1500c` 含 bit16 |
| LOWCMD → JOINTCTRL / PASSIVE | ✅ | `State_LowCmd` 出口掩码 `0x4006` 含 bit1/2 |
| LOWCMD → BACKTOSTART | ✅ | 同上掩码含 bit14 |

#### LOWCMD 载荷语义(20 字节 JointCmd)

| 字段名 | 真机含义 | 我们的组帧 |
|---|---|---|
| Pos | q 目标 | 实测位置(零位置误差) |
| W | qd 目标 | 实测速度(零速度误差) |
| T | 力矩前馈 τ_ff | clamp(command_effort, ±30/60) |
| K_P / K_W | 增益 | 0(纯前馈,关闭内部伺服) |

电机执行:`τ = Kp·(q−qm) + Kw·(q̇−q̇m) + T`

### 安全红线

1. **永远有人扶臂**,手不放
2. 力矩从 **2 N·m** 起步,确认方向后再加大
3. 任何异常:先 Ctrl-C launch(客户端停发 → PASSIVE),再断电
4. 不在臂附近放易损物;LOWCMD 下 z1_ctrl 碰撞保护行为未知

### 验证状态

| 阶段 | 结果 |
|---|---|
| B1 JOINTCTRL 基准 | ✅ |
| B2 LOWCMD 进入(两步修复后) | ✅ |
| B3 力矩通路(10 N·m → effort ≈8~14) | ✅ |
| B4 PASSIVE 退出 | ✅ |
| C2~C4 悬浮 / 关闭对照 | ✅ |

## 效果

| Gazebo Sim（`gazebo.launch.py`） | MoveIt 2（`z1_moveit_config demo.launch.py`） |
|---|---|
| ![Gazebo Sim](docs/images/gazebo_sim.png) | ![MoveIt 2](docs/images/moveit_rviz.png) |

## 设计要点

* **一份 URDF，两种 ros2_control 后端**：`hardware_plugin:=` 选
  `z1_ros2_control/Z1System`（真机，默认）或 `gz_ros2_control/GazeboSimSystem`；
  控制器集合与 MoveIt 配置完全一致。
* **Gazebo 后端用 `effort` 接口，其余用 `position`**（`z1_ros2_control.xacro` 自动切）：
  gz_ros2_control 的位置接口只有单一 P 增益、没有 D，会跟关节摩擦形成持续抖动。
* **`use_gripper` 贯穿三层**（URDF、SRDF + 控制器映射、`gripper_controller` spawner），
  三处必须同值，否则 MoveIt 会引用模型里不存在的 link。
* **[`control.launch.py`](src/z1_bringup/launch/control.launch.py) 是共用核心**，真机、
  Gazebo、MoveIt 的启动文件都组合它；两条 Gazebo 路径（gz 伺服 + bridge、
  gz_ros2_control）相互独立，不能同时运行。
* **真机上关节级伺服律留在 `z1_controller` 内部**，ros2_control 只下发位置指令。
  真机路径需要先手动起官方 `z1_ctrl` 进程（顺序和要求见
  [`z1_ros2_control/README.md`](src/z1_ros2_control/README.md)）。

## 踩坑结论

细节与实测数据见各包 README 和代码注释，这里只留结论：

1. `gripperStator` 的 joint 与 link 同名会让 `gz sim` 拒绝加载 → 固定关节改名
   `gripperStator_joint`。
2. Blender 导出的 DAE 材质在 gz/assimp 路径上丢失 → 每个 visual 显式写 `<material>`。
3. 零位/停放位姿与官方圆柱碰撞体互穿 8 mm → SRDF 中禁用 `link02`–`link06` 碰撞对。
4. `gz_ros2_control` 的 `position_proportional_gain` 写进 URDF `<plugin>` 传不到插件，
   必须写进参数文件。
5. `IncludeLaunchDescription` 会把 `launch_arguments` 泄漏到父作用域（见 `demo.launch.py`
   注释）。
6. MoveIt 的起始状态检查很苛刻：`allowed_start_tolerance` 设为 0.05；零位正好压在 joint2
   下限 / joint3 上限上，控制器激活后先限速滑到 SRDF 的 `forward` 位姿再等规划。
7. 官方命名位姿不能照抄：`home`/`startFlat` 压在 joint2 下限上，`stow` 的 joint4 超出官方
   ±87° 限位 0.052 rad，只有 `forward` 完整落在限位内——停靠位姿因此用官方 `forward`。

真机调试新出的几条（细节与实测数据见
[`z1_ros2_control/README.md`](src/z1_ros2_control/README.md)）：

8. **ros2_control 只在组件 ACTIVE 之后才调 `read()`/`write()`**
   （`AsyncComponentThread` 里显式判 lifecycle 状态）。把"等第一帧 RecvState 再发
   `SendCmd`"的握手放进 `read()` 就是死锁——激活永远超时。I/O 必须收进一个
   `pump_once()`，由生命周期回调在激活期间自己驱动。
9. URDF 里的 `rw_rate` **根本不生效**：异步硬件组件的节拍由 controller_manager 的
   `update_rate` 决定（实测：`update_rate` 250 → `read()` 250 Hz，500 → 500 Hz，两次
   URDF 都写着 500）。既然属性是死参数就删掉，让速率只在一处定义；两个速率都实测
   可用（无 overrun、z1_ctrl 无漏帧告警），当前选 250 Hz。组件会在运行时把实测速率
   打进日志，不靠配置推断。
10. 真机六个关节常驻 `error=0x40`，而 vendored 头里该位注释是 "nothing"——
    `kErrorMask` 排除它是对的；若当地真故障，真机一帧都用不了。
11. **joint2 是双电机关节**（每个 `JointState` 有两组 `Motor_State`），其余只有第一组；
    且无夹爪时第 7 个电机槽不能参与故障判定（否则健康 6 轴臂每周期 ERROR）。
12. `joint_trajectory_controller` 的单点轨迹里 `positions` 是**绝对**目标，不是增量：
    想动一个关节，必须把其余关节的当前位置一并写进轨迹点。
13. mock 写好了不等于能跑：`z1_ctrl_mock.py` 有两个致命 bug（`finally:` 里的 `return`
    吞掉所有异常；`build_reply()` 少打包 3 个字段），使它在"跑通"的样子下一包都发不
    出去。异常路径与返回值核对必须在接真机前做完。

## 来源 / 许可证

网格模型、惯量和关节限位来自
[`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
（`robots/z1_description`，**BSD-3-Clause**，© Unitree Robotics），
详见 `src/z1_description/LICENSE`。与 ROS 1 原版的差异列在
`src/z1_description/README.md` 中。

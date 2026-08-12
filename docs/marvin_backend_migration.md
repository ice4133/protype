# Marvin 中间通信后端移植说明

本次只移植应用层与 Marvin 控制层之间的后端，不接入现有 YAM
运动学，不实现不同机械臂之间的关节映射，也不自动连接真机。

已确认目标型号为 `marvin_m6_s_ccs_696_v4`。模型单独放在
`src/marvin/marvin_description`，包含标准 URDF、MuJoCo URDF 和完整 STL
资源。URDF 内的 mesh URI 已改为 `package://marvin_description/...`，不再
依赖原始 `pico_body_tianji` 包。

## 边界

- `marvin_hardware_bridge.py`：ROS 输入、状态门、安全决策和发送调度。
- `marvin_hardware.py`：SDK 会话、反馈读取和命令批量发送。
- `hardware_safety.py`、`host_readiness.py`、`home_trajectory.py`、
  `marvin_state.py`：从已验证工程一并移植的配套逻辑。
- `sdk_loader.py`：只定位随当前 ROS 包安装的厂商 SDK。

厂商的 `fx_robot.py` 和 `libMarvinSDK.so` 已复制进当前仓库，目录为：

```text
src/marvin/marvin_hardware_backend/
└── marvin_sdk/
    ├── __init__.py
    ├── fx_robot.py
    └── libMarvinSDK.so
```

`setup.py` 会把 `marvin_sdk` 和 `libMarvinSDK.so` 一起安装到当前工程的
ROS overlay。运行时不会读取或回退到 `~/tianji_teleop`，也不使用外部
SDK 路径环境变量。

厂商 `fx_robot.py` 包含 Python 3.10 才会直接求值成功的联合类型注解，
而当前 RoboStack ROS Humble 环境使用 Python 3.9。`sdk_loader.py` 在加载
SDK 时只启用 Python 自带的延迟注解解析编译标志；厂商源码和校验值保持
不变，SDK 的运行逻辑不做改写。

`VENDOR_SDK_SHA256SUMS` 记录了当前厂商 SDK 三个文件的校验值，用于确认
后续构建使用的仍是这套已核对版本。

## 当前保留的 ROS 契约

移植代码暂时保留原工程的话题与安全状态契约，避免在“移植”和“应用适配”
两个阶段同时改变行为：

```text
/pico_body_sim/left_arm/joint_commands
/pico_body_sim/right_arm/joint_commands
/pico_body/teleop_state
/pico_body/status
/pico_body_sim/status
```

这套契约将在应用层适配阶段再改成当前工程最终使用的命名、单位和机器人模型。
当前的无真机检查只验证包、入口程序和 SDK 本地加载，不发布或校验关节数据。

## 无真机检查

```bash
pixi run build-marvin
pixi run check-marvin
```

`build-marvin` 只让 `colcon` 构建并安装 `marvin_description` 和
`marvin_hardware_backend` 两个包，不启动 ROS 节点、不连接网络，也不驱动
真机。Pixi 环境来自仓库根目录现有的 `pixi.toml`；首次构建前没有
`install/setup.bash` 是正常的，`bash/pixi_activate.sh` 会跳过它，构建完成后
的后续命令才会自动 source 当前 overlay。

`check-marvin` 不调用 SDK 的 `connect()`，不访问控制器，也不发送命令。
它会检查 ROS 包和两个入口程序、导入完整真机桥依赖链、定位并加载厂商
动态库。整个检查不构造或发布任何关节数组，因此与应用层机械臂轴数无关。

## 来源

核心后端从 `~/tianji_teleop` 的提交 `cecca97` 移植。除
`marvin_hardware_bridge.py` 改为通过本包的 `sdk_loader.py` 延迟定位厂商 SDK
外，其余五个配套核心模块保持源文件内容不变，便于后续审计差异。

## 真机入口（本阶段不要执行）

第一次接入真机时使用只读反馈诊断：

```bash
pixi run marvin-feedback -- --confirm-readonly
```

默认读取 `real.yaml` 的 `robot_ip`，以 5 Hz 输出 10 秒。可选参数：

```bash
pixi run marvin-feedback -- --confirm-readonly --duration 20 --rate 2
pixi run marvin-feedback -- --confirm-readonly --robot-ip 192.168.1.190
```

该入口与正式真机桥共用独占锁，只调用厂商 SDK 的 `connect`、`subscribe`
和 `release_robot`，不清错、不使能、不设置状态、不回零、不发送关节目标。
成功标准是 A/B 两臂均返回有效关节反馈且两个 `frame_serial` 都持续变化，
最终输出 `control_commands_sent: 0`。

只读反馈通过后，第一条下行链路使用当前位置保持验收：

```bash
pixi run marvin-hold -- --confirm-hold
```

该程序不接入 IK、VR 或应用层目标。它读取进入位置模式前的实际位置，
不自动清错，以实际位置作为临时安全基准和唯一目标，并通过
`HardwareSafetyController` 决策后调用 `send_joint_targets`。默认保持 5 秒，
进入 `state=1` 前后或保持期间任一关节偏差超过 0.5 度即判失败并请求软停。
正常结束必须验证 A/B 都回到 `state=0` 后才释放 SDK。

控制器系统版本 `100343009` 在实机验收中确认：请求退出到 `state=0` 后，
A/B 可能短暂回报 `cur_state=109`、`cmd_state=-1`、`err_code=0`。后端只在
目标状态为 0 时将 109 视为受超时约束的过渡状态，并继续等待真实状态 0；
109 若持续超过 3 秒、伴随错误码，或出现在进入/保持位置模式阶段，仍判失败。

明确的成功状态为：

```json
{
  "status": "HOLD_POSITION_SUCCESS",
  "success": true,
  "safety_bridge_used": true,
  "state_0_shutdown_verified": true
}
```

任何失败返回非零退出码，并输出 `HOLD_POSITION_FAILED`、失败阶段和原因。

当前位置保持验收通过后，可以进行单臂单关节的小幅往返验收。第一次建议只使用
`1.0` 度偏移，并由现场人员根据实际扫掠空间选择手臂、关节和正负方向：

```bash
pixi run marvin-step -- \
  --confirm-motion \
  --arm A \
  --joint 7 \
  --delta-deg 1.0
```

上面的 A 臂、7 号关节和正方向只是命令格式示例，不是对现场方向的推荐。程序会
从使能前的实测位置出发，另一条手臂及其余关节保持实测位置；指定关节以零端速
平滑轨迹移动，默认单程至少 2 秒、在端点保持 1 秒，再用相同轨迹返回。试验
轨迹峰值速度限制为 1 度/秒，偏移绝对值限制为 0.1..2.0 度，所有周期目标继续
经过 `HardwareSafetyController`。目标超出 `real.yaml` 关节限制及 2 度端点余量时
直接拒绝，不会静默裁剪后继续测试。

明确的成功状态为：

```json
{
  "status": "SMALL_MOTION_SUCCESS",
  "success": true,
  "outbound_target_reached": true,
  "returned_to_reference": true,
  "safety_bridge_used": true,
  "state_0_shutdown_verified": true
}
```

程序不会回到配置 Home，也不会调用 IK。反馈失去更新、状态或错误码异常、动态
跟踪误差超过默认 0.5 度、端点误差超过默认 0.2 度时，程序请求软停并退出。

正式控制入口为：

```bash
pixi run marvin-real -- --confirm-real
```

即使提供确认参数，后端仍需收到完整且新鲜的应用状态、IK 状态和目标流，
通过启动安全门后才会连接机器人。

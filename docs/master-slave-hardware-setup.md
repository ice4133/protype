# 主从遥操作硬件适配指南

## 当前状态

4 条 YAM 臂的 CAN 接口映射已确认：

| 角色 | CAN 名称 | Serial | 说明 |
|------|----------|--------|------|
| 从臂左 | `can_yam_l` | `001E0052594E501820313332` | 原有臂 |
| 从臂右 | `can_yam_r` | `0043005C594E501820313332` | 原有臂 |
| 主臂左 | `can_master_l` | `0050003D594E501820313332` | 新臂 |
| 主臂右 | `can_master_r` | `0052003D594E501820313332` | 新臂 |

## 需要完成的步骤

### 第 1 步：确认哪条是左、哪条是右

一次只插一条臂，用 `ip link show` 确认 serial 对应的物理臂。记下：
- 哪个 serial 是 **主臂左** (master_l)
- 哪个 serial 是 **主臂右** (master_r)

如果你已经知道，直接跳到第 2 步。

### 第 2 步：配置 udev 持久命名

编辑 udev 规则文件：

```bash
sudo vim /etc/udev/rules.d/90-can-yam.rules
```

改为以下内容（保留原有的 can_yam_l/r，加上新的 master/slave）：

```
# 原有 VR 遥操作臂（保持不变）
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="001E0052594E501820313332", NAME="can_yam_l"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="0043005C594E501820313332", NAME="can_yam_r"

# 新增：主从遥操作的主臂
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="0050003D594E501820313332", NAME="can_master_l"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="0052003D594E501820313332", NAME="can_master_r"
```

> **注意：** 上面假设 `0050...` 是左主臂，`0052...` 是右主臂。根据你第 1 步的确认结果调整。

然后重新加载规则：

```bash
sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger
```

**拔掉两条新臂再重新插入**，验证命名：

```bash
ip link show | grep can_
```

应该看到 `can_master_l` 和 `can_master_r`。

### 第 3 步：决定从臂的 CAN 接口

你有两种方案：

**方案 A：原来的 2 条臂做从臂**

如果你打算用新臂做主臂（手拖）、原来的臂做从臂（跟随），修改 `config/master_slave.yaml`：

```yaml
# 主臂用新臂
/master_l:
  ros__parameters:
    arm_can_id: can_master_l   # 新臂
    ...

/master_r:
  ros__parameters:
    arm_can_id: can_master_r   # 新臂
    ...

# 从臂用原来的臂
/slave_l:
  ros__parameters:
    arm_can_id: can_yam_l      # 原来的左臂
    ...

/slave_r:
  ros__parameters:
    arm_can_id: can_yam_r      # 原来的右臂
    ...
```

**方案 B：4 条臂全部独立**

如果你以后有 4 条独立的臂，需要 4 个 USB-CAN 适配器，每个都配 udev 规则：
`can_master_l`, `can_master_r`, `can_slave_l`, `can_slave_r`。

### 第 4 步：启动 CAN 接口

每次开机或重新插拔后，需要激活 CAN 接口：

```bash
# 激活主臂
sudo ip link set can_master_l up type can bitrate 1000000
sudo ip link set can_master_r up type can bitrate 1000000

# 激活从臂（如果用原来的臂）
sudo ip link set can_yam_l up type can bitrate 1000000
sudo ip link set can_yam_r up type can bitrate 1000000
```

建议写一个脚本 `bash/activate_can_master_slave.sh`：

```bash
#!/bin/bash
sudo ip link set can_master_l up type can bitrate 1000000
sudo ip link set can_master_r up type can bitrate 1000000
sudo ip link set can_yam_l up type can bitrate 1000000
sudo ip link set can_yam_r up type can bitrate 1000000
echo "All CAN interfaces activated"
```

### 第 5 步：更新 config 并构建

根据你第 3 步选的方案，编辑 `src/yam/yam_damiao_controller/config/master_slave.yaml` 里的 `arm_can_id` 值。

然后重新构建：

```bash
pixi run colcon build --packages-select yam_damiao_controller
```

### 第 6 步：测试

建议先单独测试一条臂：

```bash
# 只启动左主臂（重力补偿模式，可以用手拖动）
pixi run ros2 launch yam_damiao_controller master_slave.launch.py \
  use_master_r:=false use_slave_l:=false use_slave_r:=false
```

确认主臂能正常 homing 并进入重力补偿后，再加从臂：

```bash
# 启动左侧一对主从
pixi run ros2 launch yam_damiao_controller master_slave.launch.py \
  use_master_r:=false use_slave_r:=false
```

最后启动全部 4 臂：

```bash
./bash/run_master_slave.sh
```

## 调参建议

| 参数 | 位置 | 说明 |
|------|------|------|
| `bilateral_enabled` | master 节 | `true`/`false` 开关力反馈 |
| `bilateral_kp_1_3` | master 节 | 力反馈刚度（大关节），建议 2~10 |
| `bilateral_kp_4_6` | master 节 | 力反馈刚度（腕关节），建议 1~5 |
| `arm_kp_1_3` | slave 节 | 从臂位置跟踪刚度，当前 120.0 |
| `gripper_torque_limit` | 两者 | 夹爪最大力矩限制 |

**安全提醒：** 首次测试时建议把 `bilateral_kp` 设低（如 2.0），确认正常后再逐步增大。

## 故障排查

| 问题 | 检查 |
|------|------|
| 臂没反应 | `ip link show` 确认 CAN 接口是 UP 状态 |
| 接口名不对 | `udevadm info -a -p /sys/class/net/canX \| grep serial` 对比 |
| homing 失败 | 确认臂开机时在零位附近（误差 < 0.1 rad） |
| 从臂不跟随 | `ros2 topic echo /master_l_status` 确认主臂在发数据 |
| 力反馈没效果 | 确认 `bilateral_enabled: true` 且 `ros2 topic echo /slave_l_status` 有数据 |

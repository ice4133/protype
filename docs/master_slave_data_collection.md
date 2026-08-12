# Master-Slave 数据采集启动指南

## 前提条件

- 已完成 `pixi install`
- 已完成 `pixi run colcon build --symlink-install`
- 四个 CAN 接口已连接（master_l, master_r, slave_l, slave_r）
- 四个 RealSense D405 相机已连接（left_wrist, right_wrist, high, top）
- USB 踏板已连接

## 启动步骤

按顺序在**不同终端**中执行以下步骤。

### 第 1 步：激活 CAN 接口

```bash
cd ~/code/prototype
sudo bash bash/activate_can_master_slave.sh
```
密码shu
配置 4 个 CAN 接口（can_master_l, can_master_r, can_slave_l, can_slave_r），波特率 1000000。

### 第 2 步：启动相机

```bash
bash bash/run_cameras.sh
```

会依次启动 3 个相机节点（cam_high, cam_top, cam_left_wrist + cam_right_wrist），间隔 5 秒，并打开 rviz2 可视化。

发布的 topic：
- `/cam_left_wrist/color/image_rect_raw` — 左腕 RGB
- `/cam_right_wrist/color/image_rect_raw` — 右腕 RGB
- `/cam_high/color/image_rect_raw` — 高位 RGB
- `/cam_high/depth/image_rect_raw` — 高位深度
- `/cam_top/color/image_rect_raw` — 顶部 RGB
- `/cam_top/depth/image_rect_raw` — 顶部深度

### 第 3 步：启动 Master-Slave 机械臂

```bash
bash bash/run_master_slave.sh
```

启动 4 条臂（master_l, master_r, slave_l, slave_r）。Master 臂为重力补偿模式（手动引导），Slave 臂跟随 Master 运动。

### 第 4 步：启动数据采集

```bash
bash bash/run_data_collector_master_slave.sh
```

启动两个节点：
- `data_collector_node` — 数据采集主节点
- `pedal_pub` — 踏板控制节点

### 第 5 步：踏板控制采集

| 踏板 | 按键 | 功能 |
|------|------|------|
| 左踏板 | `a` | 开始采集 |
| 中踏板 | `b` | 停止采集并保存 HDF5 |
| 右踏板 | `c` | subtask 标记 |

```bash
pixi run python3 bash/pedal_trigger.py
```
操作流程：
1. 踩 `a` 踏板 → 终端显示 `>>> START recording` → 开始记录
2. 操作机械臂完成动作
3. 踩 `b` 踏板 → 终端显示 `>>> STOP recording, saving...` → 停止并保存为 HDF5 文件
4. 重复 1-3 采集下一条数据

# 踩踏板时，得在bash/pedal_trigger.py这个运行的窗口才能有效

## 采集配置

### 直接修改配置路径： ~/code/prototype/install/data_collector/share/data_collector/config， 修改 data_dir 字段

| 参数 | 值 | 说明 |
|------|-----|------|
| `data_dir` | `/home/shu/data/master_slave/0316_pusht_zzy` | HDF5 输出目录 |
| `collection_frequency` | 15 Hz | 采集频率 |
| `max_frames` | 2000 | 单条最大帧数 |

### 采集的数据

| 数据 | topic | 格式 |
|------|-------|------|
| 左腕 RGB | `/cam_left_wrist/color/image_rect_raw` | uint8, 480x640x3 |
| 右腕 RGB | `/cam_right_wrist/color/image_rect_raw` | uint8, 480x640x3 |
| 高位 RGB | `/cam_high/color/image_rect_raw` | uint8, 480x640x3 |
| 顶部 RGB | `/cam_top/color/image_rect_raw` | uint8, 480x640x3 |
| 高位深度 | `/cam_high/depth/image_rect_raw` | uint16, 480x640 |
| 顶部深度 | `/cam_top/depth/image_rect_raw` | uint16, 480x640 |
| 左臂关节位置 | `/slave_l_status` | float64, [7] |
| 左臂关节速度 | `/slave_l_status` | float64, [7] |
| 左臂关节电流 | `/slave_l_status` | float64, [7] |
| 右臂关节位置 | `/slave_r_status` | float64, [7] |
| 右臂关节速度 | `/slave_r_status` | float64, [7] |
| 右臂关节电流 | `/slave_r_status` | float64, [7] |
| 踏板/subtask | `/pedal_status` | int32, [1] |

## 验证

启动后可以用以下命令检查 topic 是否正常：

```bash
# 查看所有活跃 topic
pixi run ros2 topic list

# 检查相机 RGB 是否有数据
pixi run ros2 topic hz /cam_high/color/image_rect_raw

# 检查深度图是否有数据
pixi run ros2 topic hz /cam_high/depth/image_rect_raw

# 检查踏板 topic
pixi run ros2 topic echo /pedal_status --once

# 检查机械臂状态
pixi run ros2 topic hz /slave_l_status
```

## 常见问题

- **踏板无反应**：确认 `pedal_pub` 节点已启动，终端中有 `Pedal ready` 日志
- **深度图没数据**：确认 `d405_high.py` 和 `d405_top.py` 中 `enable_depth` 为 `'true'`
- **CAN 报错**：重新执行 `sudo bash bash/activate_can_master_slave.sh`
- **相机连接失败**：检查 USB 连接，确认相机序列号与脚本中一致

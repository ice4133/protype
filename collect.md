  # 1. 激活 CAN
  sudo bash bash/activate_can_master_slave.sh
  # 密码: shu

  # 2. 启动相机
  bash bash/run_cameras.sh

  # 3. 启动主从臂控制器（新终端）
  bash bash/run_master_slave.sh

  # 4. 启动数据收集器（新终端）
  bash bash/run_data_collector_master_slave.sh

  # 5. 启动键盘触发器（新终端）
  pixi run python3 bash/collect_trigger.py
  # 按 s 开始录制，按 d 停止录制并保存，按 q 退出


  # 1. 激活 CAN
  sudo bash bash/activate_can_master_slave.sh

  # 2. 启动从臂控制器（终端1）— 监听 /YAM_VR_L,R
  pixi run ros2 launch yam_damiao_controller playback_slave.launch.py

  # 3. 启动回放（终端2）
  pixi run ros2 launch inference test_intervention_playback.launch.py \
      hdf5_path:=/home/shu/data/master_slave/collection_1773148583.hdf5 \
      loop:=false


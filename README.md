# 关闭前灯

```
cansend can0 121#0100000000000000
```

# 底盘

```
source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
rosrun tracer_bringup bringup_can2usb.bash
roslaunch tracer_bringup tracer_robot_base.launch
roslaunch tracer_bringup tracer_teleop_keyboard.launch
```

# 雷达

```
source devel/setup.bash
source devel/setup.bash
roslaunch rplidar_ros view_rplidar_a2m12.launch
```

# 里程计测试

```
source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
rosrun tracer_bringup bringup_can2usb.bash
roslaunch tracer_bringup tracer_robot_base.launch
rosbag record /odom -o odom_wheel.bag
python src/odom_test/scripts/odom_visualize.py
```

# 同步数据录制

```
source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
rosrun tracer_bringup bringup_can2usb.bash
roslaunch remote_record remote_record.launch
```


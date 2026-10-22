```
docker pull qiayuanl/unitree
```

ROS 2 in the image uses domain 1: its discovery traffic on the robot's DDS domain 0 stalls `LowState_`. ROS 2 tools on a
laptop need `export ROS_DOMAIN_ID=1`.

Quick start a terminal

```
docker run -it --rm \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
zsh
```

Auto start for Go1

```
docker run -it --name runtime \
--privileged --restart always --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch unitree_bringup real.launch.py robot_type:='go1'
```

G1 controller

```
docker run -it --rm \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch  unitree_bringup real.launch.py robot_type:='g1' network_interface:='eth0'
```

G1 LIO

```
docker run -it --rm \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch unitree_bringup lio.launch.py
```

G1 realsense auto start

```
docker run -it --name realsense --restart always \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch realsense2_camera rs_launch.py
```

G1 LIO auto start (timestamp issue, not working yet)

```
docker run -it --name lio --restart always \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch unitree_bringup lio.launch.py
```

```
docker run -it --rm \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch motion_tracking_controller real.launch.py robot_type:='g1' network_interface:="eth0" policy_path:="src/motion_tracking_controller/config/g1/spin_kick.onnx"
```

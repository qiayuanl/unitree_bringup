```
docker pull qiayuanl/unitree
```

ROS 2 in the image uses domain 1: its discovery traffic on the robot's DDS domain 0 stalls `LowState_`. ROS 2 tools on a
laptop need `export ROS_DOMAIN_ID=1`.

G1 real-time budget, once per robot: JetPack 5 kernels have `CONFIG_RT_GROUP_SCHED`, so containers, even privileged
ones, get no `SCHED_FIFO` budget, and the controller manager logs `Could not enable FIFO RT scheduling policy` and runs
as a normal thread. Add `"cpu-rt-runtime": 950000` to `/etc/docker/daemon.json`; then the controller's container,
started with `--cpu-rt-runtime=950000` as below, gets it (one running container at a time). Validate, restart docker
and check:

```
sudo dockerd --validate --config-file /etc/docker/daemon.json && sudo systemctl restart docker
docker run --rm --privileged --cpu-rt-runtime=950000 qiayuanl/unitree chrt -f 50 true
```

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
--privileged --cpu-rt-runtime=950000 --network host \
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

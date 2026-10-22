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

G1 GPU, once per robot on JetPack 5.1.1 (L4T R35.3.1; other versions need a new list): the container runtime mounts
only the driver. `tensorrt.csv` adds TensorRT 8.5, cuDNN 8.6 and the CUDA 11.4 libraries they load, and `jetson_clocks`
keeps the GPU from idling at 306 MHz between policy ticks. Install and check:

```
cd ~/colcon_ws/src/unitree_bringup/config/g1
sudo cp tensorrt.csv /etc/nvidia-container-runtime/host-files-for-container.d/
sudo cp jetson_clocks.service /etc/systemd/system/ && sudo systemctl enable --now jetson_clocks
docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all qiayuanl/unitree \
python3 -c "import ctypes; ctypes.CDLL('libnvonnxparser.so.8')"
```

With `--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all`, policies run on TensorRT (the launch log shows
`Inference backend: legged_rl_tensorrt/TensorRtBackend`) and need an engine next to each ONNX; a launch without one
fails and names this command. Build them once per policy, not while a policy runs (about 45 s of GPU load):

```
docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -v /home/unitree/colcon_ws:/colcon_ws qiayuanl/unitree \
ros2 run legged_rl_tensorrt build_engine /colcon_ws/policies/<name>_encoder.onnx /colcon_ws/policies/<name>_decoder.onnx
```

Without those flags policies run on the CPU. Use that for `wandb_path` downloads: they live in the container, so their
engines would be lost with it.

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

G1 motion tracking

```
docker run -it --rm \
--privileged --cpu-rt-runtime=950000 --network host \
--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree \
ros2 launch motion_tracking_controller real.launch.py network_interface:='eth0' \
encoder_path:='/colcon_ws/policies/<name>_encoder.onnx' decoder_path:='/colcon_ws/policies/<name>_decoder.onnx' \
motion_dir:='/colcon_ws/motions'
```

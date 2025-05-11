```
docker pull qiayuanl/unitree
```
Quick run and develop
```
docker run -it --rm \
--privileged --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree:latest \
zsh
```
Auto start
```
docker run -it --name runtime \
--privileged --restart always --network host \
-v /dev:/dev -v /home/unitree/colcon_ws:/colcon_ws \
qiayuanl/unitree:latest \
ros2 launch unitree_description real.launch.py robot_type:='go1'
```

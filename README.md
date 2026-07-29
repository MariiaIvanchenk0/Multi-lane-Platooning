# Multi-lane-Platooning
Implements Multi-lane Platooning

Terminal 1:
```
export ROS_DOMAIN_ID=32
ros2 launch multilane_formation multilane.launch.py use_rviz:=true
```

Terminal 2:
```
export ROS_DOMAIN_ID=32
ros2 launch multilane_formation platoon.launch.py
```

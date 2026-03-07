**Run Pure Pursuit**

# to run ROS simulation
```
colcon build
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

# to place the markers
```
colcon build --packages-select control
source install/setup.bash
ros2 run control marker
```

# to run pure pursuit
```
colcon build --packages-select control
source setup/install.bash
ros2 run control pure_pursuit
```
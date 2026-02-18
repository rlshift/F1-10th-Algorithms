# How to use the slam_toolbox

1. Source ros
`source /opt/ros/jazzy/setup.bash`


2. Install navigation and slam_toolbox
`sudo apt install ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-map-server`

3. Need to update the code in the gym_bridge.py
```bash
cd ~/sim_ws/src/f1tenth_gym_ros/f1tenth_gym_ros
nvim gym_bridge.py
```

4. Update the gym bridge code
Find the line (around l 112): `self.angle_inc = scan_fov / scan_beams`
And change to: `self.angle_inc = scan_fov / scan_beams`

5. Save the f1tenth_slam.yaml file from this directory

6. Run everything
In terminal 1: 
- Run simulation:
```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

In terminal 2:
- Run the SLAM toolbox
```bash
source ~/sim_ws/install/setup.bash 
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file <path to saved file>/f1tenth_slam.yaml -p use_sim_time:=true -r /map:=/slam_map
```

In terminal 3:
- Configure and Activate the slam_toolbox
```bash
source ~/sim_ws/install/setup.bash 
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

- In the simulator, go to add and add the /slam_map topic to view the SLAM output
    - Go to the actual map and set the colour scheme to costmap so it is more visible.

- Run the follow the gap algorithm
```bash
source ~/sim_ws/install/setup.bash 
cd <path to follow the gap code>
./build_follow_the_gap.bash
source install/local_setup.bash 
ros2 run follow_the_gap follow_the_gap
```

- Save the map. Once the car has finished a lap and has mapped the track, stop the follow the gap and save the map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/sim_ws/src/f1tenth_gym_ros/maps/<map_name> --fmt png
```


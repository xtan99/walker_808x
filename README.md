# walker_808x

Programming Assignment: ROS 2 Working with Gazebo
---

## Dependencies

Ros2 Humble

## Install via command line
check dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
Navigate to Workspace SRC folder
```
cd ~/ros2_ws/src
```
Clone Package
```
git clone https://github.com/xtan99/walker_808x.git
```
Build Package in Workspace
```
cd ~/ros2_ws/
colcon build --packages-select walker_808x
```
Source ros2 underlay and workspace overlay
```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/
. install/local_setup.bash
```

Launch the Turtlebot3 gazebo world and delete the turtlebot3 model 
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
Use the insert option in gazebo to navigate to cloned package and insert robot model provided

Run walker to start robot movement without recording ros2 bag
```
ros2 launch walker_808x walker_launch.xml
```

Run the following command to start robot movement and record ros2 bag and check bag output info
```
ros2 launch walker_808x walker_launch.xml b:=true
ros2 bag info bag_output
```

Run the following command to enable bag file recording(Week-11)
```
ros2 launch walker_808x launch_bag.launch.XML b:=True
ros2 bag info bag_output
```
Run the following commands to play back bag files in different terminals
```
cd ~/ros2_ws/src/walker_808x/results
ros2 bag play bag_output
```

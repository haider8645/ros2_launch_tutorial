# Introduction

Hello! I just started with ROS2 Foxy and came across some hurdles in understanding the working behind the launch files. I saw that everyone uses their 
own style for the launch file layout. I have tried here to combine the different layouts into one which works really well.

Additionally, I have added some example usages for the commonly used modules. The launch file showcases the usage of the following modules:

LaunchDescription  
Node  
FindPackageShare  
PathJoinSubstitution  
DeclareLaunchArgument  
OpaqueFunction  
Command   
FindExecutable   
LaunchConfiguration  
IfCondition  

Please feel free to add new options to this so that we have a single usecase example for the launch file capabilities.

# Layout
```
tree -d -L 2
.
├── config # config file for the joy package. Used to show how you an read configs and make decisions based on the config values
├── launch # launch file showcasing some ros2 launch capabilities
├── rviz # rviz config used to showcase the reading of such a config file and assignment to a rviz2 node
└── urdf # a rrbot urdf used to showcase how the urdf's are read using xacro and passed as parameter

4 directories
```

# Usage
```
source /opt/ros/foxy/setup.bash
git clone https://github.com/haider8645/ros2_launch_tutorial.git
colcon build --packages-select ros2_launch_tutorial --symlink-install
source install/setup.bash
ros2 launch ros2_launch_tutorial ros2.launch.py
```
if you see the following windows then everything works!
![image](https://user-images.githubusercontent.com/23505408/177525776-0068a9c5-468a-40f5-8843-c75e68a97403.png)



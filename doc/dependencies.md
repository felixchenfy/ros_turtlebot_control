(Below are the installation commands for ROS melodic. May not be complete.)

## Turtlebot
Install turtlebot3 packages ([reference](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup)).
```
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

## Gazebo
Install Gazebo for simulation([reference](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)):
```
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch # Test gazebo and turtlebot.
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch # Test teleoperating turtlebot.
```


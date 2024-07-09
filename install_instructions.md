These files are tested to work on Ubuntu 20.04 with ROS1 Noetic

Prior to Installation, please make sure the following:
1. a catkin_ws workspace is created
2. abb_driver is installed
    $ sudo apt-get install ros-noetic-abb-driver
3. abb_robot_driver is installed
    $ sudo apt-get install ros-noetic-abb-robot-driver

If a catkin_ws is not yet created, please follow [catkin_ws_tutorial](https://wiki.ros.org/catkin/Tutorials/create_a_workspace) to do so

$ cd ~/catkin_ws/src
$ git clone https://github.com/SkyWatcher-123/CMU_DFAB_RobotRoom.git
$ cd ..
$ catkin_make
$ source devel/setup.bash

# CMU SoA DFAB RobotRoom
This is the ROS setup for the current robots in the CMU SOA DFAB robot room: 
1. ABB IRB120 on Cart (stand-alone)  
  1.1. abb_irb120_support (URDF includes intel realsense d435i depth camera with camera holder on joint_5)  
  1.2. abb_irb_moveit_config  
<img src="https://github.com/SkyWatcher-123/CMU_DFAB_RobotRoom/assets/112517055/2fb3610f-83d5-4328-bc60-0838d6a9522e" alt="Description" width="300"/>
<img src="https://github.com/SkyWatcher-123/CMU_DFAB_RobotRoom/assets/112517055/3783a656-c42e-44c4-a024-0eb5eeb37e0f" alt="Description" width="300"/
>

2. ABB IRB 4400_45_196 (stand-alone)  
  2.1. abb_irb4400_45_support (Please do not use the original abb_irb4400_support package from ros_industrial!!! That package has an ABB IRB4400L/30, which is a different robot configuration. Using the wrong package can cause lethal damage!!!)  
  2.2. abb_irb4400_45_196_config  
3. ABB IRB6640 with Track 6004 (stand-alone)  
  3.1. abb_irb6640_support (note that this package is also different from the original package by ros_industrial, it includes a 6004 track)  
  3.2. abb_irb6640_moveit_config  
4. ABB IRB6640 with Track 6004 + ABB IRB 4400_45_196 (separate and combined move_group)    
  4.1.  abb_irb6640_irb4400_support  
  4.2.  dfab_irb6640_irb4400_moveit_config (with DFAB.scene)  
![image](https://github.com/SkyWatcher-123/CMU_DFAB_RobotRoom/assets/112517055/32b23b33-273d-4c6b-a0fe-8257c01389f7)  
5. DFAB_robots: a user wrapper package allowing end-effector customization, scene loadings, etc.  

Important Notes: ABB IRB4400/45 in DFAB has a IRC5 Controller with RobotWare 5.15.06, therefore it only supports abb_driver (trajectory instruction), not abb_driver (which includes RWS I/O controls and EGM)  


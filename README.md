# **eYs3D ROS SDK**

eYs3D SDK is a toolkit for eYs3D Depth camera module.

----------

## Support platforms

* Support x86_64 (64-bit) & ARM aarch64 (64-bit)
* Tested on X86 PC and NVIDIA Jetson TX2, with Ubuntu 18.04 (GCC 7.5).
* Currently only YX8062 camera module supports IMU function.

----------

## Documentations

API reference and the guide documentations.

----------

## Installation ROS Instructions

The following instructions are written for ROS Melodic on Ubuntu 18.04  

Install [ROS Melodic][1] on Ubuntu 18.04  

----------

## Link libGL.so for TX1/TX2 compile bug (Optional)

  `sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so`
  
----------
  
## Build eYs3D Robot

cd `cd eys3d_wrapper` and use the following commands to compile project:  

 - build wrapper and dm_preview package  

    `make all`   
    
 - build eys3d robot including robot base and robot control  
    
    `make robot`  

 - build eys3d robot and extra ros packages (example slam/navigation) 
    
    `make robotall`  

 - clean ros packages and ros workspace  
    
    `make cleanros`   

 - build wrapper and ros and then compress ros workspace  
    
    `make pkg`  

 - clean all  

    `make clanall`  
----------

## Usage Instructions

 - **Start the eYs3D depth camera node**
 1. After sourcing your setup bash using:  

    `source ../eys3d_ros/eys3d_ros_ws/devel/setup.bash`

 2. Create terminal and type below command:  

    `roslaunch dm_preview display.launch`  
        
 > This will stream all camera sensors and publish on the appropriate ROS topics. Other stream resolutions and frame rates can optionally be provided as parameters to the 'dm_preview.launch' file.  

 - **Set Camera Controls Using Dynamic Reconfigure Params**
 
    The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].  

    `rosrun rqt_reconfigure rqt_reconfigure`  

 - **Published Topics**  

    The published topics differ according to the device and parameters. After running the above command, the following list of topics will be available (This is a partial list. For full one type rostopic list):  

    /dm_preview/depth/camera_info  
    
    /dm_preview/depth/image_raw  
    
    /dm_preview/left/camera_info  
    
    /dm_preview/left/image_color  
    
    /dm_preview/right/camera_info  
    
    /dm_preview/right/image_color  
    
    /dm_preview/points/data_raw  
    
    /dm_preview/imu/data_raw  
    
    /dm_preview/imu/data_raw_processed  

----------
## Tank Robot operating environment setting  
1.  On the Tank Robot side, Check the IP address of Tank Robot using ‘ifconfig’ command in the terminal window or system settings.
for example, the IP address of the Tank Robot is 192.168.50.244.  
Add the ROS_HOSTNAME and ROS_MASTER_URI settings in the ‘~/.bashrc’ file as follows:  
  
    export ROS_HOSTNAME=192.168.50.244  
    export ROS_MASTER_URI=http://192.168.50.244:11311  
  
2. On remote PC side, use ‘ifconfig’ command to check the IP address of the remote PC   
For example, the IP address of remote PS is 192.168.50.236. As a precaution, 
the Tank Robot must be in the same network area as the Remote PC.  
Add the ROS_HOSTNAME and ROS_MASTER_URI settings in the ‘~/.bashrc’ file as follows:  
  
    export ROS_HOSTNAME=192.168.50.236   
    export ROS_MASTER_URI=http://192.168.50.244:11311  
  
3. Now we have completed the operating environment for the Tank Robot and remote PC. In the next section,
let’s control the Tank Robot with various ROS packages starting with remote control.
  
----------
## Execute SLAM on remote PC (SLAM) 
All the operation are on remote PC side.  
On the remote PC, use the following commands to control the Tank Robot.  
1. Open a terminal and then use ssh login into Tank Robot, type below command to bring up Tank Robot control system.  
  
    
> $ roslaunch eys3d_ros bringup.launch   
  
2. Open new terminal and use ssh login into Tank Robot, type below command to launch gmapping slam.  
   
    
> $ roslaunch eys3d_robot eys3d_lidar_slam.launch   
  
3. Open new terminal and use the following command to run roscore.  
    
    
> $ roscore   
  
4. Run the visualization tool RViz so that you can visually check the results during SLAM  
    
    
> $ rosrun rviz rviz   
  
   and select 'file' tab to open config on Rviz screen,  choose /eys3d_robot_base/rviz/slam.rviz  

5. Open a terminal and execute the below command to run twist keyboard.  
    
    
> $ rosrun teleop_twist_keyboard  teleop_twist_keyboard.py   
  
6. Now you can use above command to control the Tank Robot to perform SLAM operation manually.  
   It is importance to avoid vigorous movements such as changing the speed too quickly or rotating too fast.  
   When building a map using the Tank robot, the Tank robot should scan every corner of the environment to be measured.  
   It requires some experiences to build a clean map, so let’s practice SLAM multiple times to build up know how.  
  
7. Open a terminal and run below command to save map after you create the robust map.  
   
    
> $ cd /eys3d_robot_base/maps/   
    
> $ sh ./map.sh   
     
   The map files will be saved in the map folder. The default name is house.pgm and house.yaml.    
  
## Execute navigation on remote PC (Navigation)  
All the operations are on remote PC side. Use the following commands to control the Tank Robot on remote PC.    
1. Open a terminal and then use ssh log in to Tank Robot, type below command to bring up the Tank Robot.  
  
    
> $ roslaunch eys3d_ros bringup.launch 
  
2. Open a terminal and use ssh log in to the Tank Robot, type below command to launch move base for navigation.  
      
    
> $ roslaunch eys3d_robot eys3d_navigate.launch  
  
3. Open a terminal and use the following command to run roscore.  
      
    
> $ roscore 
  
4. Open a terminal to run rviz.  
      
    
> $ rosrun rviz rviz   

   and select 'file' tab to open config on Rviz screen,  choose /eys3d_robot_base/rviz/navigate.rviz  
   
5. Now you can execute navigation process on RViz screen.  
   First, the initial pose estimation of the Tank robot should be performed. When you press [2D Pose Estimate] in the menu of RViz,  
   a very large green arrow appears. Move it to the pose where the actual Tank robot is located in the given map,  
   and while holding down the left mouse button, drag the green arrow to the direction where the robot’s front is facing.  
   When this process is completed, the robot estimates its actual position and orientation by using the position and orientation specified by the green arrow as the initial pose.  
   When everything is ready, let’s try the move command from the navigation GUI.  
   If you press [2D Nav Goal] in the menu of RViz, a very large green arrow appears.  
   This green arrow is a marker that can specify the destination of the Tank robot.  
   Click this arrow at the position where the Tank robot will move, and drag it to set the orientation.  
   The robot will create a path to avoid obstacles to its destination based on the map.  
----------

## Install dependencies for all ROS packages (Optional)  
You can refer to the following information when you encounter problems when compiling "make robotall".  
(You can ignore this part if you just want to compile DM_Preview package on ROS workspace. ex:"make all")  

 - hector_geotiff  
`sudo apt install qt4-default`  

 - eys3d_robot  
`sudo apt-get install ros-melodic-rosserial-python`  

 - rgbdslam  
`sudo apt-get install ros-melodic-octomap`  

 - polygon_layer  
`sudo apt-get install ros-melodic-costmap-2d`  

 - robot_localization  
`sudo apt-get install ros-melodic-geographic-msgs`  

 - exploration_server  
`sudo apt-get install ros-melodic-move-base-msgs`  

 - gmapping  
`sudo apt-get install ros-melodic-openslam-gmapping`  

 - move_base  
`sudo apt-get install libsdl-image1.2-dev libsdl1.2-dev`  

 - move_base  
`sudo apt-get install ros-melodic-tf2-sensor-msgs`


----------


 ## License

This project is licensed under the [Apache License, Version 2.0](/LICENSE). Copyright 2020 eYs3D Microelectronics, Co., Ltd.


  [1]: http://wiki.ros.org/melodic/Installation/Ubuntu
# **eYs3D ROS SDK**

eYs3D ROS SDK is a toolkit for eYs3D Depth camera module.

----------

## Support platforms

* Support x86_64 (64-bit) & ARM aarch64 (64-bit)
* Tested on X86 PC and NVIDIA Jetson TX2, with Ubuntu 18.04 (GCC 7.5)
* Currently only YX8062 camera module supports IMU function
* Supports ROS Melodic on Ubuntu 18.04
* Supports ROS 1.0

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

## Usage Instructions

 - **Start the eYs3D depth camera node**
 1. Build any packages located in eys3d_ros/eys3d_ros_ws/src:
 
    `cd eys3d_ros/eys3d_ros_ws; catkin_make; cd ..`
 
 2. After sourcing your setup bash using:  

    `source ../eys3d_ros/eys3d_ros_ws/devel/setup.bash`

 3. Create terminal and type below command:  

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

 ## License

This project is licensed under the [Apache License, Version 2.0](/LICENSE). Copyright 2020 eYs3D Microelectronics, Co., Ltd.


  [1]: http://wiki.ros.org/melodic/Installation/Ubuntu

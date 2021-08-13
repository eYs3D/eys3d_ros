# **eYs3D dm_preview**

eYs3D dm_preview is a package for eYs3D Depth camera module.

----------

## Support platforms

* Support Linux x64 & ARM aarch64
* Tested on X86 PC and NVIDIA Jetson TX2, with Ubuntu 18.04 (GCC 7.5).  
* Currently only YX8062 camera module supports IMU function.  

----------

## Installation ROS Instructions

The following instructions are written for ROS Melodic on Ubuntu 18.04  

- Install [ROS Melodic][1] on Ubuntu 18.04 

- Install the dependency packages    

    `sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgtk-3-dev libusb-dev`  

    `sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libjpeg9`  

----------

## Link libGL.so for TX1/TX2 compile bug (Optional)

  `sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so`

----------

## Build dm_preview

`cd eys3d_ros_ws`  
    
`catkin_make`  

----------

## Usage Instructions

 - **Start the eYs3D dm_preview node**
 
 1. Source your setup bash using:  

    `source ./devel/setup.bash`

 2. Create terminal and type below command:  

    `roslaunch dm_preview display.launch`  
        
 > This will stream all camera sensors and publish on the appropriate ROS topics. Other stream resolutions and frame rates can optionally be provided as parameters to the 'dm_preview.launch' file.  

 - **Set Camera Controls Using Dynamic Reconfigure Params**
 
    The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].  

    `rosrun rqt_reconfigure rqt_reconfigure`   

 - **Published Topics**  

   The published topics differ according to the device and parameters. After running the above command, the following list of topics will be available.  
   
   (This is a partial list. For full one type rostopic list):  
   
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

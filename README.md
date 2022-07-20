# **eYs3D ROS SDK**

eYs3D ROS SDK is a toolkit for eYs3D Depth camera module.

----------
## Clone this project

* `git clone git@github.com:eYs3D/eys3d_ros.git --recurse-submodules`
----------
## Update submodule projects

* `git submodule update --init --recursive`
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

- Install [ROS Melodic][1] on Ubuntu 18.04  
- Install the dependency packages    

    `sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgtk-3-dev libusb-dev`  

    `sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libjpeg9` 

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

 3. Modify the 'display.launch' file based on the module information:  
    The launch file located in "eys3d_ros/eys3d_ros_ws/src/launch"  
    Please fill in the module's serial number in value field.  
    for example:  
     ` <arg name="dev_serial_number" value="8036D000000001" />  ` 
    
 4. Create terminal and type below command:  

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
## Use eYs3D camera module with rtabmap_ros package to generate SLAM (Optional)
- The rtabmap_ros package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation.
- For more information, demos and tutorials about this package, visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) page on ROS wiki.
- Install rtabmap_ros package for melodic:  
    `sudo apt install ros-melodic-rtabmap-ros`  
- To launch the example, open a terminal and launch:  
    `cd eys3d_ros/eys3d_ros_ws; catkin_make; cd ..`  
    `source ../eys3d_ros/eys3d_ros_ws/devel/setup.bash`  
    `roslaunch dm_preview rtab_demo.launch` 
----------

 ## License

This project is licensed under the [Apache License, Version 2.0](/LICENSE). Copyright 2020 eYs3D Microelectronics, Co., Ltd.


  [1]: http://wiki.ros.org/melodic/Installation/Ubuntu

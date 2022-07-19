# **eYs3D ROS SDK**

eYs3D ROS SDK is a toolkit for eYs3D Depth camera module.

----------

## Support platforms

* Support ARM aarch64 (64-bit)
* Tested on NVIDIA Jetson series, with Ubuntu 18.04 (GCC 7.5)
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

Install the dependency packages:

    `sudo apt-get install cmake git libgtk2.0-dev pkg-config  libavformat-dev libswscale-dev libv4l-dev libgtk-3-dev`
    `sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev`

----------

## Link libGL.so for TX1/TX2 compile bug (Optional)

  `sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so`

## Link opencv for Jetson Nano when cv_bridge specifies /usr/lib/opencv as include dir (Optional)

  `sudo ln -s /usr/lib/opencv4 /usr/lib/opencv`

----------

## Usage Instructions

 - **Start the eYs3D depth camera node**
 1. Build any packages located in eys3d_ros_ws/src:
 
    `cd eys3d_ros_ws; catkin_make; cd ..`
 
 2. sourcing your workspace environment using:  

    `source eys3d_ros_ws/devel/setup.bash`

 3. Create terminal and type below command depends on which camera you use :  

    `roslaunch dm_preview G100.launch`  
    `roslaunch dm_preview G53.launch`  
    `roslaunch dm_preview G50.launch`  
    `roslaunch dm_preview R50.launch`  

 4. Other stream resolutions and framerate for each camera, please see `videomode reference` folder, By change the parameter inside camera launch file.  

 5. For binding device. After launch your camera. `Serial Number` will be shown inside the terminal info. Check `dev_serial_number`parameter in the launch file. Delete annotation and change the device serial number.

 6. For multiple cameras usage. please reference `muti_camera_reference.launch`.  

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
  [2]: https://github.com/eYs3D/eys3d_ros
  [3]: https://www.ecapturecamera.com/

# **Python_sample**

Python_sample is a sample code that subscribes topic from dm_preview to display image.

----------

## Support platforms

* Support Linux x64 & ARM aarch64
* Tested on X86 PC and NVIDIA Jetson TX2, with Ubuntu 18.04 (GCC 7.5).  

----------

## Build Python_sample

`cd eys3d_ros_ws`  
    
`catkin_make`  

----------

## Usage Instructions

 - **Start the Python_sample node**
 
 1. Source your setup bash using:  

    `source ./devel/setup.bash`

 2. Create terminal and type below command to run node of dm_preview:  

    `roslaunch dm_preview dm_preview.launch`  

 3. Create terminal and type below command to run Python_sample:  

    `rosrun Python_sample Python_sample.py`  

 - **Subscribed Topics**  
      
    /dm_preview/left/image_color  -The input color image
    
    /dm_preview/depth/image_raw   -The input depth image

----------

 ## License

This project is licensed under the [Apache License, Version 2.0](/LICENSE). Copyright 2020 eYs3D Microelectronics, Co., Ltd.

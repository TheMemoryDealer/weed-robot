# CMP9767M - WEEDER
## Intro
A simulation of the Thorvald weeding robot by Saga Robotics. The solution focuses on perception and attempts to identify and eliminate weeds amongst 3 distinct types of crop: young lettuce (denoted as easy ![#2bff36](https://via.placeholder.com/15/2bff36/000000?text=+)), matured lettuce (denoted as medium ![#ffff2b](https://via.placeholder.com/15/ffff2b/000000?text=+)) and onion (denoted as hard ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+))
## Prerequisites
* Ubuntu 18.04 and ROS Melodic\
`http://wiki.ros.org/melodic/Installation/Ubuntu`

* Work is based on https://github.com/LCAS/CMP9767M. Clone it into your catkin workspace.\
`cd ~/catkin_ws && git clone https://github.com/LCAS/CMP9767M.git`

## Starter
* ALWAYS DO THIS FIRST\
`sudo apt-get update && sudo apt-get upgrade`
* Install required packages
    ```
    sudo apt-get install \
        ros-melodic-opencv-apps \
        ros-melodic-rqt-image-view \
        ros-melodic-image-geometry \
        ros-melodic-uol-cmp9767m-base \
        ros-melodic-uol-cmp9767m-tutorial \
        ros-melodic-find-object-2d \
        ros-melodic-video-stream-opencv \
        ros-melodic-image-view \
        ros-melodic-gmapping \
        ros-melodic-topological-utils \
        ros-melodic-robot-pose-publisher
    ``` 
* Rotate camera to increase FOV\
`cd CMP9767M-master/uol_cmp9767m_base/urdf`\
Where *CMP9767M-master* is the cloned LCAS repository\
`gedit sensors.xacro`\
Replace lines 268,269 with
    ```
    <xacro:property name="kinect2_cam_op" value="1.85" />
    <xacro:property name="kinect2_cam_oy" value="0" />
    ``` 
* Gazebo tips\
Restarting the simulation can be a pain. The process ***gzserver*** needs to be killed manually before attempting to reopen the simulator. The process can sometimes be stopped with `killall -9 gzserver`, but killing it through the *System Monitor* application works 100% of the time.

* Extra tips\
Using terminator (https://gnometerminator.blogspot.com/p/introduction.html) makes life much easier\
`cd && gedit .bashrc`\
and add these 3 lines bottom of the file. Your workspace is now automatically sourced with every newly opened terminal window, roscd also takes you to /catkin_ws/devel
    ```
    source /opt/ros/melodic/setup.bash
    source /home/<USERNAME>/catkin_ws/devel/setup.bash
    echo $ROS_PACKAGE_PATH
    ```
## Start topological navigation
* IF FIRST TIME USE do\
`cd && mkdir mongodb`
* Launch gazebo sim\
`roslaunch uol_cmp9767m_base thorvald-sim.launch`
* Launch move_base\
`roslaunch uol_cmp9767m_tutorial move_base_topo_nav.launch`
* Launch topological navigation\
`roslaunch uol_cmp9767m_tutorial topo_nav.launch`
* Load map into mongodb\
`rosrun topological_utils load_yaml_map.py $(rospack find weeder)/maps/map.yaml`
* Launch rviz and have a demo\
`rviz -d $(rospack find weeder)/rviz/topo.rviz`
* Finally, do this to start moving\
`rosrun weeder navigation.py`
<p align="middle">
  <img src="/weeder/assets/topo/topo.png" width="400" />
  <img src="/weeder/assets/topo/topo2.png" width="400" /> 
</p>
There are 3 main segmentation functions in /weeder/src/vision.py. Navigation node publishes crop type to topic

**thorvald_001/crop_difficulty**
, which vision subscribes to and decides which segmentation algorithm to run.

![#2bff36](https://via.placeholder.com/15/2bff36/000000?text=+) - easyAlgo(),\
![#ffff2b](https://via.placeholder.com/15/ffff2b/000000?text=+) - mediumAlgo()\
![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) - hardAlgo()\
![#000000](https://via.placeholder.com/15/000000/000000?text=+) - removeGround()\
All 3 segmentation algorithms call removeGround() on start

## Start vision
* Start vision node\
`rosrun weeder vision.py`
* Visualize with\
`rqt_image_view`

<p align="middle">
  <img src="/weeder/assets/weedseg/easy.png" width="250" />
  <img src="/weeder/assets/weedseg/medium.png" width="250" /> 
  <img src="/weeder/assets/weedseg/hard.png" width="250" />
</p>


output from **/thorvald_001/kinect2_camera/hd/image_color_rect** ^^

<p align="middle">
  <img src="/weeder/assets/weedseg/easy2.png" width="250" />
  <img src="/weeder/assets/weedseg/medium2.png" width="250" /> 
  <img src="/weeder/assets/weedseg/hard2.png" width="250" />
</p>

       ![#2bff36](https://via.placeholder.com/15/2bff36/000000?text=+) - easyAlgo() ^^      ![#ffff2b](https://via.placeholder.com/15/ffff2b/000000?text=+) - mediumAlgo() ^^        ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) - hardAlgo() ^^


output from **/thorvald_001/kinect2_camera/hd/image_color_rect_filtered** ^^

* Each of the 3 segmentation algorithms uses OpenCV **SimpleBlobDetector** to identify weeds (https://learnopencv.com/blob-detection-using-opencv-python-c/).

* World weed coordinates are obtained using everything learned in https://github.com/LCAS/CMP9767M/wiki/Workshop-4---Robot-Vision

* 

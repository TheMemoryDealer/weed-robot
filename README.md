# CMP9767M - WEEDER
Prologue: See ROScheatsheet.pdf for help, or google. http://wiki.ros.org/ROS/Tutorials <- that's quite helpful too
1. ALWAYS DO THIS FIRST\
`sudo apt-get update && sudo apt-get upgrade`
2. To run the simulator\
`roslaunch uol_cmp9767m_base thorvald-sim.launch`

`roslaunch video_stream_opencv camera.launch video_stream_provider:=/dev/video0 camera_name:=camera frame_id:=map`\
To see odometry info\
`rostopic echo /thorvald_001/odometry/base_raw `\
To see what camera is seeing\
`rqt_image_view`
# Start topo nav
IF FIRST TIME USE do\
`cd && mkdir mongodb`\
Launch gazebo sim\
`roslaunch uol_cmp9767m_base thorvald-sim.launch`\
Launch move_base\
`roslaunch uol_cmp9767m_tutorial move_base_topo_nav.launch`\
Launch topological navigation\
`roslaunch uol_cmp9767m_tutorial topo_nav.launch`\
Load map into mongodb\
`rosrun topological_utils load_yaml_map.py $(rospack find weeder)/maps/map.yaml`\
Launch rviz and have a demo\
`rviz -d $(rospack find uol_cmp9767m_tutorial)/config/topo_nav.rviz`
<p align="middle">
  <img src="/weeder/assets/topo/topo.png" width="400" />
  <img src="/weeder/assets/topo/topo2.png" width="400" /> 
</p>
There's 3 main segmentation funcs in /weeder/src/vision.py. Navigation node publishes crop type to topic

**thorvald_001/crop_difficulty**
, which vision subscribes to and decides which segmentation algorithm to run.

![#2bff36](https://via.placeholder.com/15/2bff36/000000?text=+) - easyAlgo(),\
![#ffff2b](https://via.placeholder.com/15/ffff2b/000000?text=+) - mediumAlgo()\
![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) - hardAlgo()\
![#000000](https://via.placeholder.com/15/000000/000000?text=+) - removeGround()\
All 3 segmentation algos call removeGround() on start

# Start vision
Start vision node\
`rosrun weeder vision.py`\
Visualize with\
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

           ![#2bff36](https://via.placeholder.com/15/2bff36/000000?text=+) - easyAlgo() ^^         ![#ffff2b](https://via.placeholder.com/15/ffff2b/000000?text=+) - mediumAlgo() ^^      ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) - hardAlgo() ^^


output from **/thorvald_001/kinect2_camera/hd/image_color_rect_filtered** ^^

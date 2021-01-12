# CMP9767M
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
# Start vision
<p align="middle">
  <img src="/weeder/assets/weedseg/easy.png" width="300" />
  <img src="/weeder/assets/weedseg/medium.png" width="300" /> 
  <img src="/weeder/assets/weedseg/hard.png" width="300" />
</p>
<p align="middle">
  <img src="/weeder/assets/weedseg/easy2.png" width="300" />
  <img src="/weeder/assets/weedseg/medium2.png" width="300" /> 
  <img src="/weeder/assets/weedseg/hard2.png" width="300" />
</p>

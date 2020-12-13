# CMP9767M
Prologue: See ROScheatsheet.pdf for help, or google. http://wiki.ros.org/ROS/Tutorials <- that's quite helpful too
1. ALWAYS DO THIS FIRST\
`sudo apt-get update && sudo apt-get upgrade`
2. To run the simulator\
`roslaunch uol_cmp9767m_base thorvald-sim.launch`

`roslaunch video_stream_opencv camera.launch video_stream_provider:=/dev/video0 camera_name:=camera frame_id:=map
`

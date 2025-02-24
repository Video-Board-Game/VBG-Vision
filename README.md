# VBG-Vision

To run on Raspberry Pi: 
`ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=424,240,15 depth_module.exposure:=8000 enable_sync:=true pointcloud.enable:=true enable_color:=true initial_reset:=true rgb_camera.color_profile:=424,240,15 align_depth.enable:=true reconnect_timeout:=15.`

WSL networking 
https://learn.microsoft.com/en-us/windows/wsl/networking

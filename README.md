# VBG-Vision

To run on Raspberry Pi: 
`ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=424,240,15 depth_module.exposure:=8000 enable_sync:=true pointcloud.enable:=true enable_color:=true initial_reset:=true rgb_camera.color_profile:=424,240,15 align_depth.enable:=true reconnect_timeout:=15.`

WSL networking 
https://learn.microsoft.com/en-us/windows/wsl/networking

Patch kernel:
mesa drivers
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#prerequisites
consider: https://github.com/IntelRealSense/realsense-ros/issues/3158
https://github.com/IntelRealSense/librealsense/issues/9931#issuecomment-964289692


https://github.com/IntelRealSense/realsense-ros/issues/3158 ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15 global_time_enabled:=false enable_sync:=true initial_reset:=true reconnect_timeout:=15.0


https://github.com/IntelRealSense/realsense-ros/issues/3158#issuecomment-2268817314

ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15 global_time_enabled:=false enable_sync:=true initial_reset:=true reconnect_timeout:=15.0 pointcloud.enable:=true align_depth.enable:=true enable_color:=true

ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=424x240x6 rgb_camera.color_profile:=424x240x6 depth_module.infra_profile:=424x240x6 global_time_enabled:=false enable_sync:=true initial_reset:=true reconnect_timeout:=15.0 pointcloud.enable:=true align_depth.enable:=true enable_color:=true

ros2 topic pub /detected_object_centroid geometry_msgs/Point "{x: 424, y: 120, z: 0}"
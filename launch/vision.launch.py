from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for all parameters (dynamic)
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log verbosity level'),
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/camera/depth/color/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic', default_value='/detected_object_centroid', description='2D centroid target coordinates topic'),
        DeclareLaunchArgument("centroid_topic", default_value="/detected_centroid", description="Topic for 3D Point from VBM extract_cluster"),
        DeclareLaunchArgument('camera_info_topic_depth', default_value='/camera/camera/aligned_depth_to_color/camera_info', description='Camera depth image info topic'),
        DeclareLaunchArgument('camera_info_topic_color', default_value='/camera/camera/color/camera_info', description='Camera color image info topic'),
        DeclareLaunchArgument('camera_depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw', description='Camera depth image topic'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of filters and normals'),
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.005', description='RANSAC distance threshold'),
        DeclareLaunchArgument('header_frame', default_value='camera_color_optical_frame', description='Frame of reference for the camera point cloud color'),
        DeclareLaunchArgument('header_frame_arm', default_value='arm_frame', description='Frame of reference for the arm'),
        DeclareLaunchArgument('header_frame_depth', default_value='camera_depth_optical_frame', description='Frame of reference for the camera point cloud depth'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance'),
        DeclareLaunchArgument('curvature', default_value='0.01', description='Curvature value for edge detection'),
        DeclareLaunchArgument('normal_search_radius', default_value='0.03', description='Normal search radius'),
        DeclareLaunchArgument("state_topic",default_value="state",description="Topic to publish task manager state information"),
        Node(
            package='vbg-vision',
            executable='vision',
            name='vbg_vision',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'coord_topic': LaunchConfiguration('coord_topic'),
                'centroid_topic':LaunchConfiguration('centroid_topic'),
                'camera_info_topic_depth': LaunchConfiguration('camera_info_topic_depth'),
                'camera_info_topic_color': LaunchConfiguration('camera_info_topic_color'),
                'camera_depth_topic': LaunchConfiguration('camera_depth_topic'),
                'visualize': LaunchConfiguration('visualize'),
                'crop_radius': LaunchConfiguration('crop_radius'),
                'sor_mean_k': LaunchConfiguration('sor_mean_k'),
                'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
                'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                'ransac_max_iterations': LaunchConfiguration('ransac_max_iterations'),
                'ransac_distance_threshold': LaunchConfiguration('ransac_distance_threshold'),
                'header_frame': LaunchConfiguration('header_frame'),
                'header_frame_arm': LaunchConfiguration('header_frame_arm'),
                'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                'max_cluster_size': LaunchConfiguration('max_cluster_size'),
                'target_point_tolerance': LaunchConfiguration('target_point_tolerance'),
                'curvature': LaunchConfiguration('curvature'),
                'normal_search_radius': LaunchConfiguration('normal_search_radius'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub",
            # Pitch rotation 30 deg + translation
            arguments = ['--x', '0.1', '--y', '0', '--z', '-0.16', '--yaw', '0', '--pitch', '0.523599', '--roll', '0', '--frame-id', 'arm_frame', '--child-frame-id', 'camera_link']
        ),
        IncludeLaunchDescription(
            FindPackageShare('realsense2_camera').find('realsense2_camera') + '/launch/rs_launch.py',
            launch_arguments=
                {
                    "depth_module.depth_profile":"424,240,15",
                    "depth_module.exposure":"8000",
                    "enable_sync":"true",
                    "pointcloud.enable":"true",
                    "enable_color":"true",
                    "initial_reset":"true",
                    "rgb_camera.color_profile":"424,240,15",
                    "align_depth.enable":"true",
                    "reconnect_timeout":"15.",
                }.items() 
        ),
    ])
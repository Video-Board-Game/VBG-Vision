from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # -------------------------------------- VISION NODE --------------------------------------
        # Flags for debugging
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log verbosity level'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of PCL filtered clouds and normals'),
        DeclareLaunchArgument('extract', default_value='false',description='Switch between extracting cluster in 3D to get centroid (true, more accurate but runs slower) and just doing 2D->3D conversion (false)'),

        # Topic configuration
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/camera/depth/color/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic_start', default_value='/click_2d/start', description='2D center click target coordinates start topic'),
        DeclareLaunchArgument('coord_topic_goal', default_value='/click_2d/goal', description='2D center click target coordinates goal topic'),
        DeclareLaunchArgument("centroid_start_topic", default_value="/detected_centroid/start", description="Topic for 3D Point from VBG extract_cluster start"),
        DeclareLaunchArgument("centroid_goal_topic", default_value="/detected_centroid/goal", description="Topic for 3D Point from VBG extract_cluster goal"),
        DeclareLaunchArgument('camera_info_topic_depth', default_value='/camera/camera/aligned_depth_to_color/camera_info', description='Camera depth image info topic'),
        DeclareLaunchArgument('camera_info_topic_color', default_value='/camera/camera/color/camera_info', description='Camera color image info topic'),
        DeclareLaunchArgument('camera_depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw', description='Camera depth image topic'),
        DeclareLaunchArgument('pos_topic', default_value='/grasp_pose', description='Grasp pose topic'),

        # Header frame names
        DeclareLaunchArgument('header_frame', default_value='camera_color_optical_frame', description='Frame of reference for the camera point cloud color'),
        DeclareLaunchArgument('header_frame_arm', default_value='arm_frame', description='Frame of reference for the arm'),
        DeclareLaunchArgument('header_frame_depth', default_value='camera_depth_optical_frame', description='Frame of reference for the camera point cloud depth'),

        # PCL filter arguments for extract cluster node
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.005', description='RANSAC distance threshold'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance in m'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size in number of points'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size in number of points'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance in m'),

        # Grasp stability metrics for optimal grasp node
        DeclareLaunchArgument('select_stability_metric', default_value='1', description='1: maximum minimum svd, 2: maximum volume ellipsoid in wrench space,\
                               3: isotropy index, 4: maximum minimum svd with abs for numeric stability, 5: weighing (1) and (2) equally'),
        DeclareLaunchArgument('variance_neighbors', default_value='4', description='Grasp uncertainty variance neighbors to search'),
        DeclareLaunchArgument('variance_threshold', default_value='0.2', description='Grasp uncertainty variance threshold'),

        # ---------------------------------------- WEB API ----------------------------------------
        DeclareLaunchArgument('host',default_value='mcalec.dyn.wpi.edu',description="WebSocket host name / IP"),
        DeclareLaunchArgument('port',default_value='8000',description="WebSocket host port number"),
        DeclareLaunchArgument('camera_image_topic',default_value='/camera/camera/color/image_raw',description="Camera image topic for UI"),

        Node(
            package='vbg-vision',
            executable='vision',
            name='vbg_vision',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'coord_topic_start': LaunchConfiguration('coord_topic_start'),
                'coord_topic_goal': LaunchConfiguration('coord_topic_goal'),
                'centroid_start_topic':LaunchConfiguration('centroid_start_topic'),
                'centroid_goal_topic':LaunchConfiguration('centroid_goal_topic'),
                'camera_info_topic_depth': LaunchConfiguration('camera_info_topic_depth'),
                'camera_info_topic_color': LaunchConfiguration('camera_info_topic_color'),
                'camera_depth_topic': LaunchConfiguration('camera_depth_topic'),
                'visualize': LaunchConfiguration('visualize'),
                'extract': LaunchConfiguration('extract'),
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
                    "depth_module.depth_profile":"424,240,6",
                    "rgb_camera.color_profile":"424,240,6",
                    "depth_module.infra_profile":"424,240,6",
                    # "depth_module.exposure":"8000",
                    "enable_sync":"true",
                    "pointcloud.enable":"true",
                    "enable_color":"true",
                    "initial_reset":"true",
                    "align_depth.enable":"true",
                    "reconnect_timeout":"15.",
                    "global_time_enabled":"false",
                }.items() 
        ),
        Node(
            package='web_api',
            executable='web_api',
            name='ros2_web_bridge',
            output='screen',
            parameters=[{
                'host':LaunchConfiguration('host'),
                'port':LaunchConfiguration('port'),
                'coord_topic_start':LaunchConfiguration('coord_topic_start'),
                'coord_topic_goal':LaunchConfiguration('coord_topic_goal'),
                'camera_image_topic':LaunchConfiguration('camera_image_topic'),

            }]
        ),
        Node(
            package='state_machine',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{
                'centroid_start_topic':LaunchConfiguration('centroid_start_topic'),
                'centroid_goal_topic':LaunchConfiguration('centroid_goal_topic'),
                # TODO add more as needed
            }]
        ),
    ])
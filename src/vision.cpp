/*
Vision pipeline to extract object clusters. 

Subscribes to camera depth cloud and separate topic broadcasting center of target object in 2D.
Convert 2D coordinates to 3D points in point cloud. Use Euclidean clustering to detect clusters
and publish the object containing the target 3D point.

Special thanks to: 
https://github.com/IntelRealSense/librealsense/issues/11031#issuecomment-1352879033
https://support.intelrealsense.com/hc/en-us/community/posts/24972964701331--finding-3d-coordinates-of-a-given-point-which-is-specified-in-the-2d-pixel-coordinates [NOT USED]
https://github.com/yehengchen/DOPE-ROS-D435 [NOT USED]
https://medium.com/@pacogarcia3/calculate-x-y-z-real-world-coordinates-from-image-coordinates-using-opencv-from-fdxlabs-0adf0ec37cef [TODO]

*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
// #include <pcl/features/boundary.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/convex_hull.h>
// #include <pcl/features/principal_curvatures.h>
#include <Eigen/Core>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
// #include <sstream>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

class PointCloudClusterDetector : public rclcpp::Node {
public:
    PointCloudClusterDetector() : Node("vbg_vision") {
        // ROS parameters
        this->declare_parameter<std::string>("pointcloud_topic", "/camera/camera/depth/color/points");
        this->declare_parameter<std::string>("coord_topic", "/detected_object_centroid");
        this->declare_parameter<std::string>("cluster_topic", "/detected_cluster");
        this->declare_parameter<std::string>("centroid_topic","/extract_centroid");
        this->declare_parameter<std::string>("camera_info_topic_depth", "/camera/camera/aligned_depth_to_color/camera_info");
        this->declare_parameter<std::string>("camera_info_topic_color", "/camera/camera/color/camera_info");
        this->declare_parameter<std::string>("camera_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<bool>("visualize", false);
        this->declare_parameter<bool>("extract", false);
        this->declare_parameter<double>("crop_radius", 0.03);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.01);
        this->declare_parameter<int>("ransac_max_iterations", 1000);
        this->declare_parameter<double>("ransac_distance_threshold", 0.01);
        this->declare_parameter<std::string>("header_frame","camera_color_optical_frame"); // TODO: get this directly from subscribed topic
        this->declare_parameter<std::string>("header_frame_arm","arm_frame");
        this->declare_parameter<std::string>("header_frame_depth","camera_depth_optical_frame");
        this->declare_parameter<double>("cluster_tolerance", 0.02);
        this->declare_parameter<int>("min_cluster_size", 100);
        this->declare_parameter<int>("max_cluster_size", 25000);
        this->declare_parameter<double>("target_point_tolerance",0.02);
        this->declare_parameter<std::string>("state_topic","state");

        // Retrieve ROS parameters
        pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
        coord_topic = this->get_parameter("coord_topic").as_string();
        cluster_topic = this->get_parameter("cluster_topic").as_string();
        centroid_topic = this->get_parameter("centroid_topic").as_string();
        camera_info_topic_depth = this->get_parameter("camera_info_topic_depth").as_string();
        camera_info_topic_color = this->get_parameter("camera_info_topic_color").as_string();
        camera_depth_topic = this->get_parameter("camera_depth_topic").as_string();
        VISUALIZE = this->get_parameter("visualize").as_bool();
        EXTRACT = this->get_parameter("extract").as_bool();
        crop_radius = this->get_parameter("crop_radius").as_double();
        sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        sor_stddev_mul_thresh = this->get_parameter("sor_stddev_mul_thresh").as_double();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();
        ransac_distance_threshold = this->get_parameter("ransac_distance_threshold").as_double();
        header_frame = this->get_parameter("header_frame").as_string();
        header_frame_arm = this->get_parameter("header_frame_arm").as_string(); 
        header_frame_depth = this->get_parameter("header_frame_depth").as_string();
        cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size = this->get_parameter("max_cluster_size").as_int();
        target_point_tolerance = this->get_parameter("target_point_tolerance").as_double();
        state_topic = this->get_parameter("state_topic").as_string();

        // Subscriptions + Publishers 
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10, std::bind(&PointCloudClusterDetector::pointcloud_callback, this, std::placeholders::_1));
        coord_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            coord_topic, 10, std::bind(&PointCloudClusterDetector::coord_callback, this, std::placeholders::_1));
        camera_info_depth_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_depth, 10, 
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Camera depth image width: %d, height: %d", msg->width, msg->height);
                image_width_depth_ = msg->width;
                image_height_depth_ = msg->height;
                fx_ = msg->k[0];  // Focal length x
                fy_ = msg->k[4];  // Focal length y
                cx_ = msg->k[2];  // Optical center x
                cy_ = msg->k[5];  // Optical center y
                // Unsubscribe after receiving the data once
                camera_info_depth_subscription_.reset();
            });
        camera_info_color_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_color, 10, 
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Camera color image width: %d, height: %d", msg->width, msg->height);
                image_width_color_ = msg->width;
                image_height_color_ = msg->height;
                fx_ = msg->k[0];  // Focal length x
                fy_ = msg->k[4];  // Focal length y
                cx_ = msg->k[2];  // Optical center x
                cy_ = msg->k[5];  // Optical center y
                // Unsubscribe after receiving the data once
                camera_info_color_subscription_.reset();
            });
        camera_depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_depth_topic, 10, std::bind(&PointCloudClusterDetector::depth_img_callback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            state_topic, 10, std::bind(&PointCloudClusterDetector::state_callback,this, std::placeholders::_1));
        cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cluster_topic, 10);
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(centroid_topic,10);
        if(VISUALIZE){
            point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_coords", 10);
            crop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/crop", 10);
            sor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/sor", 10);
            voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/voxel", 10);
            plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/plane", 10);        
        }

        // Transforms
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    std::string pointcloud_topic, coord_topic, cluster_topic, camera_info_topic_depth, camera_info_topic_color, camera_depth_topic, header_frame, 
        header_frame_arm, header_frame_depth, state_topic, state, centroid_topic;
    double crop_radius, sor_stddev_mul_thresh, voxel_leaf_size, ransac_distance_threshold, cluster_tolerance, target_point_tolerance;
    double fx_, fy_, cx_, cy_;
    bool VISUALIZE = false, EXTRACT = false;
    int sor_mean_k, ransac_max_iterations, min_cluster_size, max_cluster_size;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coord_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_color_subscription_, camera_info_depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_depth_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_, crop_pub_, sor_pub_, voxel_pub_, plane_pub_;    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_, centroid_pub_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_data_;
    sensor_msgs::msg::Image::SharedPtr depth_img_data_;
    std::pair<int, int> latest_2d_point_;
    int image_width_depth_, image_height_depth_, image_width_color_, image_height_color_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void state_callback(const std_msgs::msg::String::SharedPtr msg){
        state = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received State: '%s'", state.c_str());
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Point cloud received!");
        pointcloud_data_ = msg;
    }

    void depth_img_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_DEBUG(this->get_logger(), "Depth image received!");
        depth_img_data_ = msg;
    }

    void coord_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_2d_point_ = std::make_pair(static_cast<int>(msg->x), static_cast<int>(msg->y));
        RCLCPP_DEBUG(this->get_logger(), "New 2D point received: (%d, %d)", latest_2d_point_.first, latest_2d_point_.second);

        if (pointcloud_data_ && depth_img_data_ && image_width_depth_ && image_width_color_) {
            auto t1 = std::chrono::high_resolution_clock::now();
            process_coordinates();
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double,std::milli> elapsed = t2-t1;
            RCLCPP_INFO(this->get_logger(),"Cluster_time elapsed from receiving point: %f",elapsed.count());            
        } else {
            RCLCPP_WARN(this->get_logger(), "No point cloud or no depth image received yet or missing depth / color camera info");
        }
    }

    /*
    // Function to compute the average depth within a window around (u, v)
    double get_average_depth(const cv::Mat& depth_image, int u, int v, int window_size = 5) {
        int valid_count = 0;
        double depth_sum = 0.0;

        // Iterate over the window of size +/- window_size around (u, v)
        for (int i = u - window_size; i <= u + window_size; ++i) {
            for (int j = v - window_size; j <= v + window_size; ++j) {
                // Check if (i, j) is within bounds of the image
                if (i >= 0 && i < depth_image.cols && j >= 0 && j < depth_image.rows) {
                    uint16_t depth_value = depth_image.at<uint16_t>(j, i);
                    if (depth_value > 0) { // Only consider non-zero depth values
                        depth_sum += depth_value;
                        valid_count++;
                    }
                }
            }
        }

        // If there are no valid depth values, return 0 (or you could handle this case differently)
        if (valid_count == 0) {
            return 0.0;
        }

        // Return the average depth value (converted to meters if necessary)
        return (depth_sum / valid_count) * 0.001; // Assuming depth values are in mm, convert to meters
    }*/

    void process_coordinates() {
        // Get 2D coordinate in color image and convert to depth image
        int u = latest_2d_point_.first * image_width_depth_/image_width_color_;
        int v = latest_2d_point_.second * image_height_depth_/image_height_color_;

        // Convert ROS to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(depth_img_data_, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch(cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        // Depth value
        float depth_scale = 0.001;  // Convert from mm to meters
        double depth_value = cv_ptr->image.at<uint16_t>(v,u) * depth_scale;
        RCLCPP_DEBUG(this->get_logger(), "Depth %.3f with point (%d, %d)",depth_value, u, v);
        // double depth_value = get_average_depth(cv_ptr->image, u, v);

        // Retrieve the 3D point corresponding to the 2D coordinates
        // Pinhole camera model
        if(depth_value < 0.15){ // check for erroneous results
            RCLCPP_WARN(this->get_logger(), "Invalid depth %.3f with point (%d, %d)",depth_value, u, v);
            // Print the depth matrix
            // RCLCPP_DEBUG(this->get_logger(), "Depth image:\n%s", (std::ostringstream() << cv_ptr->image).str().c_str());
            return;
        }
        double x = (u - cx_) * depth_value / fx_;
        double y = (v - cy_) * depth_value / fy_;
        pcl::PointXYZ target_point = pcl::PointXYZ(x,y,depth_value);

        // Transform 3D point to arm frame
        geometry_msgs::msg::PointStamped msg, msg_tf2;
        msg.header.frame_id = header_frame;
        msg.header.stamp.nanosec = 0;
        msg.header.stamp.sec = 0;
        msg.point.x = x;
        msg.point.y = y;
        msg.point.z = depth_value;
        msg_tf2 = transform_point(msg);
        target_point.x = msg_tf2.point.x;
        target_point.y = msg_tf2.point.y;
        target_point.z = msg_tf2.point.z;

        // Publish 3D point
        if(VISUALIZE){
            point_pub_->publish(msg_tf2);
        }

        if(!EXTRACT){
            centroid_pub_->publish(msg_tf2);
        }
        else{

            RCLCPP_INFO(this->get_logger(), "Converted 3D Point: (%.3f, %.3f, %.3f)", target_point.x, target_point.y, target_point.z);


            // Convert PointCloud2 to PCL PointCloud and transform to arm frame
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointcloud_data_, *cloud_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(transform_point_cloud(cloud_)));


            // Filter cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);

            // Crop cloud to region around point of interest
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(cloud);
            float radius = crop_radius; // meters
            crop.setMin(Eigen::Vector4f(target_point.x - radius, target_point.y - radius, target_point.z - radius, 0));
            crop.setMax(Eigen::Vector4f(target_point.x + radius, target_point.y + radius, target_point.z + radius, 0));
            crop.filter(*cloud_crop);
            RCLCPP_DEBUG(this->get_logger(), "Crop box found with %lu points", cloud_crop->points.size());

            // Remove noise using a Statistical Outlier Removal filter
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_crop);
            sor.setMeanK(sor_mean_k);  // Number of neighbors to analyze for each point
            sor.setStddevMulThresh(sor_stddev_mul_thresh);  // Standard deviation multiplier
            sor.filter(*cloud_sor);
            RCLCPP_DEBUG(this->get_logger(), "SOR filtered, remaining cloud found with %lu points", cloud_sor->points.size());

            // Downsample point cloud using VoxelGrid filter
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud_sor);
            vg.setLeafSize(voxel_leaf_size,voxel_leaf_size,voxel_leaf_size); 
            vg.filter(*cloud_voxel);
            RCLCPP_DEBUG(this->get_logger(), "Voxel sampled with %lu points", cloud_voxel->points.size());

            // Remove ground (plane segmentation)
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (ransac_max_iterations); 
            seg.setDistanceThreshold (ransac_distance_threshold);
            // Segment the largest planar component from the cropped cloud
            seg.setInputCloud (cloud_voxel);
            seg.segment (*inliers, *coeff);
            // coefficients = coeff; // Store plane coefficients if desired
            if (inliers->indices.size () == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to estimate planar model");
            }
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_voxel);
            extract.setIndices(inliers);
            extract.setNegative (true); // false = return plane
            extract.filter (*cloud_processed);
            RCLCPP_DEBUG(this->get_logger(), "Plane removed, cloud found with %lu points", cloud_processed->points.size());

            if(VISUALIZE){
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed_inverted(new pcl::PointCloud<pcl::PointXYZ>);
                // Invert remove ground (plane segmentation)
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
                seg.setOptimizeCoefficients (true);
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setMaxIterations (ransac_max_iterations);
                seg.setDistanceThreshold (ransac_distance_threshold);
                // Segment the largest planar component from the cropped cloud
                seg.setInputCloud (cloud_voxel);
                seg.segment (*inliers, *coeff);
                // coefficients = coeff; // Store plane coefficients if desired
                if (inliers->indices.size () == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to estimate planar model");
                }
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_voxel);
                extract.setIndices(inliers);
                extract.setNegative (false); // false = return plane
                extract.filter (*cloud_processed_inverted);

                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud_crop, cloud_msg);
                cloud_msg.header.frame_id = header_frame_arm; 
                cloud_msg.header.stamp = this->now();
                crop_pub_->publish(cloud_msg); 


                pcl::toROSMsg(*cloud_sor, cloud_msg);
                cloud_msg.header.frame_id = header_frame_arm; 
                cloud_msg.header.stamp = this->now();
                sor_pub_->publish(cloud_msg);

                pcl::toROSMsg(*cloud_voxel,cloud_msg);
                cloud_msg.header.frame_id = header_frame_arm; 
                cloud_msg.header.stamp = this->now();            
                voxel_pub_->publish(cloud_msg);

                pcl::toROSMsg(*cloud_processed_inverted,cloud_msg);
                cloud_msg.header.frame_id = header_frame_arm; 
                cloud_msg.header.stamp = this->now();
                plane_pub_->publish(cloud_msg);
            }

            // Find the object cluster containing this point
            auto cluster = find_object_cluster(cloud_processed, target_point);
            if (cluster) {
                RCLCPP_INFO(this->get_logger(), "Cluster found with %lu points", cluster->points.size());
                // Trigger optimal_grasp node if state is grasping
                if(VISUALIZE){ // TODO
                    sensor_msgs::msg::PointCloud2 cluster_msg;
                    pcl::toROSMsg(*cluster, cluster_msg);
                    cluster_msg.header.frame_id = header_frame_arm;
                    cluster_msg.header.stamp = this->now();
                    cluster_pub_->publish(cluster_msg); 
                }
                // Publish centroid regardless of state if this node was called
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cluster, centroid);
                geometry_msgs::msg::PointStamped centroid_msg;
                centroid_msg.header.frame_id = header_frame_arm;
                centroid_msg.header.stamp.nanosec = 0;
                centroid_msg.header.stamp.sec = 0;
                centroid_msg.point.x = centroid[0];
                centroid_msg.point.y = centroid[1];
                centroid_msg.point.z = centroid[2];

                centroid_pub_->publish(centroid_msg);
            } else {
                // if no cluster, publish 2D->3D point as approximate target
                RCLCPP_INFO(this->get_logger(), "No cluster :/");
                geometry_msgs::msg::PointStamped point_msg;
                point_msg.header.frame_id = header_frame_arm;
                point_msg.header.stamp.nanosec = 0;
                point_msg.header.stamp.sec = 0;
                point_msg.point.x = target_point.x;
                point_msg.point.y = target_point.y;
                point_msg.point.z = target_point.z;
                centroid_pub_->publish(point_msg);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr find_object_cluster(
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
            const pcl::PointXYZ &target_point) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);

        // Set up clustering parameters
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_filtered);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance); // 2cm
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        
        // Perform clustering
        std::vector<pcl::PointIndices> cluster_indices;
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);
        RCLCPP_DEBUG(this->get_logger(), "Clustering found something with %lu indices", cluster_indices.size());

        // Check if target point is in any of the clusters
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (int index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }

            // Check if target_point is in this cluster
            double tolerance = target_point_tolerance;
            for (const auto& point : cluster->points) {
                if (std::fabs(point.x - target_point.x) < tolerance &&
                    std::fabs(point.y - target_point.y) < tolerance &&
                    std::fabs(point.z - target_point.z) < tolerance) {
                    return cluster;
                }
            }
        }
        return nullptr;
    }

    geometry_msgs::msg::TransformStamped get_transform(std::string target_frame, std::string source_frame){
        try {
            // Lookup the transform from child_frame to parent_frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame, // i.e. world or arm frame
                source_frame, // i.e. camera frame
                rclcpp::Time(0) // latest transform
            );

            return transform;
        }
        catch (const tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not find transform: %s", e.what());
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                source_frame, 
                source_frame, 
                rclcpp::Time(0) // latest transform
            );
            return transform;
        }
    }

    // Transform a point from the camera's point frame ("camera_depth_optical_frame") to the arm frame ("arm_frame") 
    geometry_msgs::msg::PointStamped transform_point(geometry_msgs::msg::PointStamped msg){
        // Lookup the transform from child_frame to parent_frame
        geometry_msgs::msg::TransformStamped transform = get_transform(header_frame_arm, msg.header.frame_id);

        // Transform the point from the child frame to the parent frame
        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(msg, transformed_point, transform);

        RCLCPP_DEBUG(this->get_logger(), "Header frame: %s", msg.header.frame_id.c_str());
        RCLCPP_DEBUG(this->get_logger(), "Original point: (%.3f, %.3f, %.3f); Transformed point: (%.3f, %.3f, %.3f)",
                    msg.point.x, msg.point.y, msg.point.z,
                    transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

        return transformed_point;
    }

    // Transform a point cloud from the camera's point frame ("camera_depth_optical_frame") to the arm frame ("arm_frame") 
    pcl::PointCloud<pcl::PointXYZ> transform_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> output_cloud;

        for (const auto& point : input_cloud->points){
            geometry_msgs::msg::PointStamped point_in;
            point_in.header.frame_id = header_frame_depth; 
            point_in.header.stamp = rclcpp::Time(0);  // Use the latest time
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;

            // Transform the point from child frame to parent frame
            geometry_msgs::msg::PointStamped point_out = transform_point(point_in);

            // Add the transformed point to the output cloud
            pcl::PointXYZ pcl_point;
            pcl_point.x = point_out.point.x;
            pcl_point.y = point_out.point.y;
            pcl_point.z = point_out.point.z;

            output_cloud.points.push_back(pcl_point);
        }

        // Set the output cloud header and other properties
        output_cloud.width = output_cloud.points.size();
        output_cloud.height = 1;  // Assuming unorganized point cloud
        output_cloud.is_dense = true;

        return output_cloud;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudClusterDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
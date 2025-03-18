#ifndef SMALL_GICP_LOCALIZATION__SMALL_GICP_LOCALIZATION_HPP_
#define SMALL_GICP_LOCALIZATION__SMALL_GICP_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace small_gicp_localization
{
    class SmallGicpLocalization : public rclcpp::Node
    {
        public:
        explicit SmallGicpLocalization(const rclcpp::NodeOptions & options);

        private:
        int num_threads;
        int num_neighbors;
        float global_leaf_size;
        float registered_leaf_size;
        float max_dist_sq;
        std::string map_frame;
        std::string odom_frame;
        std::string base_link_frame;
        std::string pcd_file;
        std::string cloud_registered_topic;
        std::string cloud_frame_id;
        bool ready;

        pcl::PointCloud<pcl::PointCovariance>::Ptr PCD_downsampled;
        pcl::PointCloud<pcl::PointCovariance>::Ptr cloud_downsampled;

        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> PCD_tree;
        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> cloud_tree;
        std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP>> localization_register;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub;
        rclcpp::Time last_cloud_stamp;
        rclcpp::TimerBase::SharedPtr publish_timer;
        rclcpp::TimerBase::SharedPtr localization_timer;

        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

        Eigen::Isometry3d init_T;
        Eigen::Isometry3d publish_T;

        void load_pcd_file(const std::string& pcd_file);
        void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
        void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
        void publish_transform();
        void perform_localization();
    };
}

#endif
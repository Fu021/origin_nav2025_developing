#ifndef LIDAR_TRANSFORM__LIDAR_TRANSFORM_HPP_
#define LIDAR_TRANSFORM__LIDAR_TRANSFORM_HPP_

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Dense>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace lidar_transform
{
    class LidarTransform : public rclcpp::Node
    {
        public:
        explicit LidarTransform(const rclcpp::NodeOptions& options);

        private:
        std::string lidar_custom_topic;
        std::string imu_topic;
        std::string lidar_pointcloud_topic;
        std::string lidar_frame;
        std::string base_link_frame;
        std::string lidar_custom_topic_pub;
        std::string imu_topic_pub;
        std::string lidar_pointcloud_topic_pub;

        message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_custom_sub;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_pointcloud_sub;
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<livox_ros_driver2::msg::CustomMsg,sensor_msgs::msg::Imu,sensor_msgs::msg::PointCloud2>;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

        rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_custom_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pointcloud_pub;

        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;

        void lidar_and_imu_callback(
            const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg1, 
            const sensor_msgs::msg::Imu::ConstSharedPtr& msg2,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg3
        );
        Eigen::Matrix3d toEigenMatrix(const std::array<double, 9>& covariance);
        std::array<double, 9> toCovarianceArray(const Eigen::Matrix3d& matrix);
    };
}

#endif
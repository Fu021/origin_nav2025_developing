#include <lidar_transform/lidar_transform.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

namespace lidar_transform
{
    LidarTransform::LidarTransform(const rclcpp::NodeOptions& options)
    : Node("lidar_transform", options)
    {
        this->declare_parameter<std::string>("lidar_custom_topic","/livox/lidar");
        this->declare_parameter<std::string>("imu_topic","/livox/imu");
        this->declare_parameter<std::string>("lidar_frame","mid360");
        this->declare_parameter<std::string>("base_link_frame","base_link");
        this->declare_parameter<std::string>("lidar_custom_topic_pub","/base_link/lidar");
        this->declare_parameter<std::string>("imu_topic_pub","/base_link/imu");

        this->get_parameter("lidar_custom_topic",lidar_custom_topic);
        this->get_parameter("imu_topic",imu_topic);
        this->get_parameter("lidar_frame",lidar_frame);
        this->get_parameter("base_link_frame",base_link_frame);
        this->get_parameter("lidar_custom_topic_pub",lidar_custom_topic_pub);
        this->get_parameter("imu_topic_pub",imu_topic_pub);

        lidar_custom_sub.subscribe(this, lidar_custom_topic);
        imu_sub.subscribe(this, imu_topic);

        sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100),lidar_custom_sub,imu_sub
        );
        sync->registerCallback(
            std::bind(&LidarTransform::lidar_and_imu_callback,this,std::placeholders::_1,std::placeholders::_2)
        );

        lidar_custom_pub = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
            lidar_custom_topic_pub,10
        );
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_pub,10
        );

        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }

    void LidarTransform::lidar_and_imu_callback(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg1, 
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg2
    )
    {
        geometry_msgs::msg::TransformStamped lidar_to_base_link;
        livox_ros_driver2::msg::CustomMsg msg1_pub;
        sensor_msgs::msg::Imu msg2_pub;

        try
        {
            lidar_to_base_link = buffer->lookupTransform(base_link_frame,lidar_frame,rclcpp::Time(0));
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(),"Could't lookup transform from %s to %s: %s",
                lidar_frame.c_str(),base_link_frame.c_str(),e.what());
            return;
        }

        Eigen::Affine3d lidar_to_base_link_eigen = tf2::transformToEigen(lidar_to_base_link);
        
        // 转换点云
        msg1_pub.header = msg1->header;
        msg1_pub.header.frame_id = base_link_frame;
        msg1_pub.timebase = msg1->timebase;
        msg1_pub.point_num = msg1->point_num;
        msg1_pub.lidar_id = msg1->lidar_id;
        msg1_pub.rsvd = msg1->rsvd;
        for (size_t i = 0; i < msg1->points.size(); ++i)
        {
            Eigen::Vector3d point(msg1->points[i].x,msg1->points[i].y,msg1->points[i].z);
            point = lidar_to_base_link_eigen * point;

            livox_ros_driver2::msg::CustomPoint new_point = msg1->points[i];
            new_point.x = point.x();
            new_point.y = point.y();
            new_point.z = point.z();

            msg1_pub.points.push_back(new_point);
        }
        
        // 转换imu
        msg2_pub.header.stamp = msg1->header.stamp;
        msg2_pub.header.frame_id = base_link_frame;

        Eigen::Matrix3d rotation = lidar_to_base_link_eigen.rotation();
        //
        Eigen::Vector3d linear_acceleration(
            msg2->linear_acceleration.x,
            msg2->linear_acceleration.y,
            msg2->linear_acceleration.z
        );

        Eigen::Vector3d angular_velocity(
            msg2->angular_velocity.x,
            msg2->angular_velocity.y,
            msg2->angular_velocity.z
        );
        Eigen::Vector3d transformed_acceleration = rotation * linear_acceleration;
        Eigen::Vector3d transformed_velocity = rotation * angular_velocity;

        msg2_pub.linear_acceleration.x = transformed_acceleration.x();
        msg2_pub.linear_acceleration.y = transformed_acceleration.y();
        msg2_pub.linear_acceleration.z = transformed_acceleration.z();

        msg2_pub.angular_velocity.x = transformed_velocity.x();
        msg2_pub.angular_velocity.y = transformed_velocity.y();
        msg2_pub.angular_velocity.z = transformed_velocity.z();

        Eigen::Matrix3d linear_cov = toEigenMatrix(msg2->linear_acceleration_covariance);
        Eigen::Matrix3d updated_linear_cov = rotation * linear_cov * rotation.transpose();
        msg2_pub.linear_acceleration_covariance = toCovarianceArray(updated_linear_cov);

        Eigen::Matrix3d angular_cov = toEigenMatrix(msg2->angular_velocity_covariance);
        Eigen::Matrix3d updated_angular_cov = rotation * angular_cov * rotation.transpose();
        msg2_pub.angular_velocity_covariance = toCovarianceArray(updated_angular_cov);

        Eigen::Quaterniond orientation_quat(
            msg2->orientation.w,
            msg2->orientation.x,
            msg2->orientation.y,
            msg2->orientation.z
        );

        Eigen::Quaterniond transformed_orientation = Eigen::Quaterniond(rotation) * orientation_quat;

        msg2_pub.orientation.w = transformed_orientation.w();
        msg2_pub.orientation.x = transformed_orientation.x();
        msg2_pub.orientation.y = transformed_orientation.y();
        msg2_pub.orientation.z = transformed_orientation.z();

        lidar_custom_pub->publish(msg1_pub);
        imu_pub->publish(msg2_pub);
    }

    Eigen::Matrix3d LidarTransform::toEigenMatrix(const std::array<double, 9>& covariance)
    {
        Eigen::Matrix3d matrix;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix(i, j) = covariance[i * 3 + j];
            }
        }
        return matrix;
    }

    std::array<double, 9> LidarTransform::toCovarianceArray(const Eigen::Matrix3d& matrix)
    {
        std::array<double, 9> covariance;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance[i * 3 + j] = matrix(i, j);
            }
        }
        return covariance;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_transform::LidarTransform)
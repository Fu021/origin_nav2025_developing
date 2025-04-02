#include <lidar_transform/lidar_transform.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <ctime>

namespace lidar_transform
{
    LidarTransform::LidarTransform(const rclcpp::NodeOptions &options)
        : Node("lidar_transform", options)
    {
        this->declare_parameter<std::string>("lidar_custom_topic", "/livox/lidar");
        this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("lidar_pointcloud_topic", "/livox/lidar/pointcloud");
        this->declare_parameter<std::string>("lidar_frame", "mid360");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("lidar_custom_topic_pub", "/base_link/lidar");
        this->declare_parameter<std::string>("imu_topic_pub", "/base_link/imu");
        this->declare_parameter<std::string>("lidar_pointcloud_topic_pub", "/base_link/lidar/pointcloud");

        this->get_parameter("lidar_custom_topic", lidar_custom_topic);
        this->get_parameter("imu_topic", imu_topic);
        this->get_parameter("lidar_pointcloud_topic", lidar_pointcloud_topic);
        this->get_parameter("lidar_frame", lidar_frame);
        this->get_parameter("base_link_frame", base_link_frame);
        this->get_parameter("lidar_custom_topic_pub", lidar_custom_topic_pub);
        this->get_parameter("imu_topic_pub", imu_topic_pub);
        this->get_parameter("lidar_pointcloud_topic_pub", lidar_pointcloud_topic_pub);

        // 重入回调组
        lidar_custom_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        imu_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        lidar_pointcloud_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions lidar_custom_options;
        lidar_custom_options.callback_group = lidar_custom_group;
        lidar_custom_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_custom_topic, 10,
            std::bind(&LidarTransform::lidar_custom_callback, this, std::placeholders::_1),
            lidar_custom_options);

        rclcpp::SubscriptionOptions lidar_pointcloud_options;
        lidar_pointcloud_options.callback_group = lidar_pointcloud_group;
        lidar_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_pointcloud_topic, 10,
            std::bind(&LidarTransform::lidar_pointcloud_callback, this, std::placeholders::_1),
            lidar_pointcloud_options);

        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_group;
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&LidarTransform::imu_callback, this, std::placeholders::_1),
            imu_options);

        lidar_custom_pub = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
            lidar_custom_topic_pub, 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_pub, 10);
        lidar_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            lidar_pointcloud_topic_pub, 10);

        listen_transform_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&LidarTransform::listen_transfrom_callback, this));

        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }

    void LidarTransform::listen_transfrom_callback()
    {
        geometry_msgs::msg::TransformStamped lidar_to_base_link;
        try
        {
            lidar_to_base_link = buffer->lookupTransform(base_link_frame, lidar_frame, rclcpp::Time(0));
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could't lookup transform from %s to %s: %s",
                        lidar_frame.c_str(), base_link_frame.c_str(), e.what());
            return;
        }
        // 加锁更新变换矩阵
        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            lidar_to_base_link_eigen_3d = tf2::transformToEigen(lidar_to_base_link);
            lidar_to_base_link_eigen_3f = lidar_to_base_link_eigen_3d.cast<float>();
        }
    }

    void LidarTransform::lidar_custom_callback(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
    {
        livox_ros_driver2::msg::CustomMsg msg_pub;

        // 转换点云
        msg_pub.header = msg->header;
        msg_pub.header.frame_id = base_link_frame;
        msg_pub.timebase = msg->timebase;
        msg_pub.point_num = msg->point_num;
        msg_pub.lidar_id = msg->lidar_id;
        msg_pub.rsvd = msg->rsvd;
        // 加锁读取变换矩阵
        Eigen::Affine3f transform;
        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            transform = lidar_to_base_link_eigen_3f;
        }
        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            Eigen::Vector3f point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
            point = transform * point;

            livox_ros_driver2::msg::CustomPoint new_point = msg->points[i];
            new_point.x = point.x();
            new_point.y = point.y();
            new_point.z = point.z();

            msg_pub.points.push_back(new_point);
        }

        lidar_custom_pub->publish(msg_pub);
    }

    void LidarTransform::lidar_pointcloud_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        sensor_msgs::msg::PointCloud2 msg_pub;

        pointcloud_transform(*msg, msg_pub);
        msg_pub.header.stamp = msg->header.stamp;
        msg_pub.header.frame_id = base_link_frame;

        lidar_pointcloud_pub->publish(msg_pub);
    }

    void LidarTransform::pointcloud_transform(
        const sensor_msgs::msg::PointCloud2 &p_in, sensor_msgs::msg::PointCloud2 &p_out)
    {
        p_out = p_in;

        sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, std::string("x"));
        sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, std::string("y"));
        sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, std::string("z"));

        sensor_msgs::PointCloud2Iterator<float> x_out(p_out, std::string("x"));
        sensor_msgs::PointCloud2Iterator<float> y_out(p_out, std::string("y"));
        sensor_msgs::PointCloud2Iterator<float> z_out(p_out, std::string("z"));
        // 加锁读取变换矩阵
        Eigen::Affine3f transform;
        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            transform = lidar_to_base_link_eigen_3f;
        }
        Eigen::Vector3f point;
        for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
        {
            point = lidar_to_base_link_eigen_3f * Eigen::Vector3f(*x_in, *y_in, *z_in);
            *x_out = point.x();
            *y_out = point.y();
            *z_out = point.z();
        }
    }

    void LidarTransform::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        sensor_msgs::msg::Imu msg_pub;

        msg_pub.header.stamp = msg->header.stamp;
        msg_pub.header.frame_id = base_link_frame;

        Eigen::Matrix3d rotation = lidar_to_base_link_eigen_3d.rotation();
        //
        Eigen::Vector3d linear_acceleration(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        Eigen::Vector3d angular_velocity(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        Eigen::Vector3d transformed_acceleration = rotation * linear_acceleration;
        Eigen::Vector3d transformed_velocity = rotation * angular_velocity;

        msg_pub.linear_acceleration.x = transformed_acceleration.x();
        msg_pub.linear_acceleration.y = transformed_acceleration.y();
        msg_pub.linear_acceleration.z = transformed_acceleration.z();

        msg_pub.angular_velocity.x = transformed_velocity.x();
        msg_pub.angular_velocity.y = transformed_velocity.y();
        msg_pub.angular_velocity.z = transformed_velocity.z();

        Eigen::Matrix3d linear_cov = toEigenMatrix(msg->linear_acceleration_covariance);
        Eigen::Matrix3d updated_linear_cov = rotation * linear_cov * rotation.transpose();
        msg_pub.linear_acceleration_covariance = toCovarianceArray(updated_linear_cov);

        Eigen::Matrix3d angular_cov = toEigenMatrix(msg->angular_velocity_covariance);
        Eigen::Matrix3d updated_angular_cov = rotation * angular_cov * rotation.transpose();
        msg_pub.angular_velocity_covariance = toCovarianceArray(updated_angular_cov);

        Eigen::Quaterniond orientation_quat(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);

        Eigen::Quaterniond transformed_orientation = Eigen::Quaterniond(rotation) * orientation_quat;

        msg_pub.orientation.w = transformed_orientation.w();
        msg_pub.orientation.x = transformed_orientation.x();
        msg_pub.orientation.y = transformed_orientation.y();
        msg_pub.orientation.z = transformed_orientation.z();

        imu_pub->publish(msg_pub);
    }

    Eigen::Matrix3d LidarTransform::toEigenMatrix(const std::array<double, 9> &covariance)
    {
        Eigen::Matrix3d matrix;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                matrix(i, j) = covariance[i * 3 + j];
            }
        }
        return matrix;
    }

    std::array<double, 9> LidarTransform::toCovarianceArray(const Eigen::Matrix3d &matrix)
    {
        std::array<double, 9> covariance;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                covariance[i * 3 + j] = matrix(i, j);
            }
        }
        return covariance;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_transform::LidarTransform)
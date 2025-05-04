#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/float32.hpp>

#include <eigen3/Eigen/Geometry>

class DipAngle : public rclcpp::Node
{
public:
    DipAngle()
        : Node("dip_angle")
    {
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "Odometry",10,std::bind(&DipAngle::odom_callback,this,std::placeholders::_1)
        );

        dip_angle_pub = this->create_publisher<std_msgs::msg::Float32>(
            "dip_angle",10
        );
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z
        );
        Eigen::Vector3d local_z(0, 0, 1);
        Eigen::Vector3d rotated_z = q * local_z;

        Eigen::Vector3d world_z(0, 0, 1);

        double cos_theta = rotated_z.dot(world_z) / (rotated_z.norm() * world_z.norm());
        double angle_rad = std::acos(cos_theta);
        double angle_deg = angle_rad * 180.0 / M_PI;

        std_msgs::msg::Float32 msg1;
        msg1.data = angle_deg;
        dip_angle_pub->publish(msg1);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dip_angle_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DipAngle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
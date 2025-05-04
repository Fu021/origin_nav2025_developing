#include <rm_serial_cpp/rm_serial_cpp.hpp>

namespace rm_serial_cpp
{
    RmSerialCpp::RmSerialCpp(const rclcpp::NodeOptions & options)
    : Node("rm_serial_cpp", options)
    {
        auto sensor_data_qos = rclcpp::SensorDataQoS();

        autoaim_target_sub = this->create_subscription<rm_interfaces::msg::Target>(
            "/armor_solver/target", sensor_data_qos, std::bind(&RmSerialCpp::autoaim_target_callback, this, std::placeholders::_1)
        );
        autoaim_gimbal_cmd_sub = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
            "/armor_solver/cmd_gimbal", sensor_data_qos, std::bind(&RmSerialCpp::autoaim_gimbal_cmd_callback, this, std::placeholders::_1)
        );
        nav_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_chassis", sensor_data_qos, std::bind(&RmSerialCpp::nav_cmd_vel_callback, this, std::placeholders::_1)
        );
        nav_rot_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/nav_rotate", 10, std::bind(&RmSerialCpp::nav_rot_callback, this, std::placeholders::_1)
        );
        nav_yaw_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/nav_yaw", 10, std::bind(&RmSerialCpp::nav_yaw_callback, this, std::placeholders::_1)
        );
        nav_pitch_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/nav_pitch", 10, std::bind(&RmSerialCpp::nav_pitch_callback, this, std::placeholders::_1)
        );

        referee_pub = this->create_publisher<referee_msg::msg::Referee>(
            "/referee",10
        );
        autoaim_gimbal_pub = this->create_publisher<rm_interfaces::msg::Gimbal>(
            "/gimbal_status",10
        );

        send_all_timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 150.0),std::bind(&RmSerialCpp::send_all_callback,this)
        );

        serial = std::make_shared<Serial>(*this);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    void RmSerialCpp::autoaim_target_callback(const rm_interfaces::msg::Target::SharedPtr msg)
    {
        autoaim_tracking.store(msg->tracking);
    }

    void RmSerialCpp::autoaim_gimbal_cmd_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
    {
        autoaim_fire_advice.store(msg->fire_advice);
        autoaim_yaw.store(msg->yaw);
        autoaim_pitch.store(msg->pitch);
    }

    void RmSerialCpp::nav_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        nav_vx.store(msg->linear.x);
        nav_vy.store(msg->linear.y);
    }

    void RmSerialCpp::nav_rot_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        nav_rot.store(msg->data);
    }

    void RmSerialCpp::nav_yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        nav_yaw.store(msg->data);
    }

    void RmSerialCpp::nav_pitch_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        nav_pitch.store(msg->data);
    }

    void RmSerialCpp::referee_pub_publish(const referee_msg::msg::Referee& msg)
    {
        referee_pub->publish(msg);
    }

    void RmSerialCpp::autoaim_gimbal_pub_publish(const rm_interfaces::msg::Gimbal& msg)
    {
        autoaim_gimbal_pub->publish(msg);
    }

    void RmSerialCpp::send_all_callback()
    {
        AllData all_data(
            nav_vx.load(), nav_vy.load(), nav_yaw.load(), nav_rot.load(), nav_pitch.load(), 
            autoaim_tracking.load(), autoaim_fire_advice.load(), autoaim_yaw.load(), autoaim_pitch.load()
        );
        serial->send_all(all_data);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_cpp::RmSerialCpp)
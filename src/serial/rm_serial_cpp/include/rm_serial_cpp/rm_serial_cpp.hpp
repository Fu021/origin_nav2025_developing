#ifndef RM_SERIAL_CPP__RM_SERIAL_CPP_HPP_
#define RM_SERIAL_CPP__RM_SERIAL_CPP_HPP_

#include <rclcpp/rclcpp.hpp>

#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/gimbal.hpp>

#include <referee_msg/msg/referee.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>

#include <rm_serial_cpp/Serial.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace rm_serial_cpp
{
    class Serial;

    struct AllData
    {
        double nav_vx;
        double nav_vy;
        float nav_yaw;
        int nav_rot;
        bool nav_pitch;
        bool autoaim_tracking;
        bool autoaim_fire_advice;
        double autoaim_yaw;
        double autoaim_pitch;

        AllData(double nav_vx,
        double nav_vy,
        float nav_yaw,
        int nav_rot,
        bool nav_pitch,
        bool autoaim_tracking,
        bool autoaim_fire_advice,
        double autoaim_yaw,
        double autoaim_pitch)
        : nav_vx(nav_vx), nav_vy(nav_vy), nav_yaw(nav_yaw), nav_rot(nav_rot), nav_pitch(nav_pitch),
        autoaim_tracking(autoaim_tracking), autoaim_fire_advice(autoaim_fire_advice), 
        autoaim_yaw(autoaim_yaw), autoaim_pitch(autoaim_pitch) {}
    };

    class RmSerialCpp : public rclcpp::Node
    {
        public:
            explicit RmSerialCpp(const rclcpp::NodeOptions & options);

            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

            void referee_pub_publish(const referee_msg::msg::Referee& msg);
            void autoaim_gimbal_pub_publish(const rm_interfaces::msg::Gimbal& msg);
        private:
            std::shared_ptr<rm_serial_cpp::Serial> serial;

            std::atomic<double> nav_vx{0.0};
            std::atomic<double> nav_vy{0.0};
            std::atomic<float> nav_yaw{0.0};
            std::atomic<int> nav_rot{0};
            std::atomic<bool> nav_pitch{false};

            std::atomic<bool> autoaim_tracking{false};
            std::atomic<bool> autoaim_fire_advice{false};
            std::atomic<double> autoaim_yaw{0.0};
            std::atomic<double> autoaim_pitch{0.0};

            rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr autoaim_target_sub;
            rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr autoaim_gimbal_cmd_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_vel_sub;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nav_rot_sub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr nav_yaw_sub;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_pitch_sub;

            rclcpp::Publisher<referee_msg::msg::Referee>::SharedPtr referee_pub;
            rclcpp::Publisher<rm_interfaces::msg::Gimbal>::SharedPtr autoaim_gimbal_pub;

            rclcpp::TimerBase::SharedPtr send_all_timer;

            void autoaim_target_callback(const rm_interfaces::msg::Target::SharedPtr msg);
            void autoaim_gimbal_cmd_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
            void nav_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void nav_rot_callback(const std_msgs::msg::Int32::SharedPtr msg);
            void nav_yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);
            void nav_pitch_callback(const std_msgs::msg::Bool::SharedPtr msg);
            void send_all_callback();
    };
}

#endif
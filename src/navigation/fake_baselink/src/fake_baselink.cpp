#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FakeBaselink : public rclcpp::Node
{
public:
    FakeBaselink(const rclcpp::NodeOptions & options)
        : Node("fake_baselink", options)
    {
        this->declare_parameter("odom_frame_id","odom");
        this->declare_parameter("base_link_frame_id","base_link");
        this->declare_parameter("base_link_fake_frame_id","base_link_fake");
        this->declare_parameter("cmd_vel_topic_name","cmd_vel");
        this->declare_parameter("cmd_vel_after_topic_name","cmd_vel_chassis");
        this->declare_parameter("forward_distance",0.2);

        this->get_parameter("odom_frame_id",odom_frame_id);
        this->get_parameter("base_link_frame_id",base_link_frame_id);
        this->get_parameter("base_link_fake_frame_id",base_link_fake_frame_id);
        this->get_parameter("cmd_vel_topic_name",cmd_vel_topic_name);
        this->get_parameter("cmd_vel_after_topic_name",cmd_vel_after_topic_name);
        this->get_parameter("forward_distance",forward_distance);

        tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer);
        tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_name,1,std::bind(&FakeBaselink::cmd_vel_callback,this,std::placeholders::_1)
        );
        // local_plan_sub = this->create_subscription<nav_msgs::msg::Path>(
        //     "/local_plan",1,std::bind(&FakeBaselink::local_plan_callback,this,std::placeholders::_1)
        // );

        cmd_vel_after_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_after_topic_name,10
        );

        pub_fake_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),std::bind(&FakeBaselink::pub_fake_callback,this)
        );

        init_base_link_fake();
    }

    void init_base_link_fake()
    {
        geometry_msgs::msg::TransformStamped odom_to_base_link,fake_trans;
        while (1)
        {
            try
            {
                odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
                break;
            }
            catch(...){}
        }
        fake_q = odom_to_base_link.transform.rotation;
        base_link_translation = odom_to_base_link.transform.translation;

        fake_trans.header.stamp = this->get_clock()->now();
        fake_trans.header.frame_id = odom_frame_id;
        fake_trans.child_frame_id = base_link_fake_frame_id;

        fake_trans.transform.translation = base_link_translation;
        fake_trans.transform.rotation = fake_q;

        tf2_broadcaster->sendTransform(fake_trans);
        ready = 1;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        // if (local_plan_last.empty())
        // {
        //     return;
        // }

        geometry_msgs::msg::TransformStamped odom_to_base_link;
        try
        {
            odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
        }
        catch(...){}

        // base_link_translation = odom_to_base_link.transform.translation;
        // float x1 = odom_to_base_link.transform.translation.x;
        // float y1 = odom_to_base_link.transform.translation.y;
        // float x2 = local_plan_last[local_plan_last.size()-1].pose.position.x;
        // float y2 = local_plan_last[local_plan_last.size()-1].pose.position.y;
        // float base_link_to_last_point_distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
        // size_t index = local_plan_last.size() - 1;
        // for (size_t i = 0; i < local_plan_last.size(); ++i)
        // {
        //     if (local_plan_last_distance[i] + forward_distance <= base_link_to_last_point_distance)
        //     {
        //         index = i;
        //         break;
        //     }
        // }

        // char log[1000];
        // sprintf(log,"all: %d choose: %d",local_plan_last.size(),index);
        // RCLCPP_INFO(this->get_logger(),log);

        //fake_q = local_plan_last[index].pose.orientation;

        double base_link_angle = tf2::getYaw(odom_to_base_link.transform.rotation);
        double angle_diff = base_link_angle - tf2::getYaw(fake_q);

        geometry_msgs::msg::Twist cmd_vel_after;
        cmd_vel_after.angular.z = (cmd_vel->angular.z != 0) ? 2.0 : 0;
        cmd_vel_after.linear.x = cmd_vel->linear.x * cos(angle_diff) + cmd_vel->linear.y * sin(angle_diff);
        cmd_vel_after.linear.y = -cmd_vel->linear.x * sin(angle_diff) + cmd_vel->linear.y * cos(angle_diff);

        cmd_vel_after_pub->publish(cmd_vel_after);
        
    }

    // void local_plan_callback(const nav_msgs::msg::Path::SharedPtr local_plan)
    // {
    //     if (local_plan->poses.empty())
    //     {
    //         return;
    //     }

    //     local_plan_last = std::vector<geometry_msgs::msg::PoseStamped>(local_plan->poses);

    //     local_plan_last_distance.clear();
    //     double x1 = local_plan_last[local_plan_last.size()-1].pose.position.x;
    //     double y1 = local_plan_last[local_plan_last.size()-1].pose.position.y;
    //     for (size_t i = 0; i < local_plan_last.size(); ++i)
    //     {
    //         double x2 = local_plan_last[i].pose.position.x;
    //         double y2 = local_plan_last[i].pose.position.y;

    //         local_plan_last_distance.push_back(sqrt(pow(x1-x2,2)+pow(y1-y2,2)));
    //     }
    // }

    void pub_fake_callback()
    {
        if (!ready)
            return;

        geometry_msgs::msg::TransformStamped fake_trans,odom_to_base_link;

        try
        {
            odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
            base_link_translation = odom_to_base_link.transform.translation;
        }
        catch(...){}

        fake_trans.header.stamp = this->get_clock()->now();
        fake_trans.header.frame_id = odom_frame_id;
        fake_trans.child_frame_id = base_link_fake_frame_id;

        fake_trans.transform.translation = base_link_translation;
        fake_trans.transform.rotation = fake_q;

        tf2_broadcaster->sendTransform(fake_trans);

    }

private:
    std::string odom_frame_id;
    std::string base_link_frame_id;
    std::string base_link_fake_frame_id;
    std::string cmd_vel_topic_name;
    std::string cmd_vel_after_topic_name;
    
    std::int8_t ready;

    float forward_distance;

    geometry_msgs::msg::Quaternion fake_q;
    geometry_msgs::msg::Vector3 base_link_translation;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_after_pub;

    rclcpp::TimerBase::SharedPtr pub_fake_timer;

    std::vector<geometry_msgs::msg::PoseStamped> local_plan_last;
    std::vector<float> local_plan_last_distance;
};

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<FakeBaselink>("fake_baselink");
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FakeBaselink)
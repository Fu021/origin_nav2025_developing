// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "potential_escape_action/potential_escape_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/utils.h>

namespace nav2_behavior_tree
{

  std::optional<double> EscapeAction::calculate_escape_angle(
      const nav_msgs::msg::OccupancyGrid &costmap,
      double radius,
      const std::string &robot_frame)
  {
    if (!tf_buffer_)
    {
      RCLCPP_ERROR(node_->get_logger(), "TF buffer not initialized");
      return std::nullopt;
    }

    try
    {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          costmap.header.frame_id, robot_frame, tf2::TimePointZero);

      const double res = costmap.info.resolution;
      const int width = costmap.info.width;
      const int height = costmap.info.height;
      const double origin_x = costmap.info.origin.position.x;
      const double origin_y = costmap.info.origin.position.y;

      const double x_robot = transform.transform.translation.x;
      const double y_robot = transform.transform.translation.y;
      const int mx = static_cast<int>((x_robot - origin_x) / res);
      const int my = static_cast<int>((y_robot - origin_y) / res);
      const int radius_cells = static_cast<int>(radius / res);

      const int x_start = std::max(0, mx - radius_cells);
      const int x_end = std::min(width, mx + radius_cells + 1);
      const int y_start = std::max(0, my - radius_cells);
      const int y_end = std::min(height, my + radius_cells + 1);

      double total_force_x = 0.0;
      double total_force_y = 0.0;
      const double epsilon = 1e-6;

      for (int y = y_start; y < y_end; ++y)
      {
        for (int x = x_start; x < x_end; ++x)
        {
          const int8_t cost = costmap.data[y * width + x];
          if (cost <= 0)
            continue;

          const double dx = (mx - x) * res;
          const double dy = (my - y) * res;
          const double dist_sq = dx * dx + dy * dy + epsilon;
          const double weight = static_cast<double>(cost);

          total_force_x += weight * dx / (dist_sq * std::sqrt(dist_sq));
          total_force_y += weight * dy / (dist_sq * std::sqrt(dist_sq));
        }
      }

      const double norm = std::hypot(total_force_x, total_force_y);
      if (norm < 1e-6)
        return std::nullopt;

      const double global_angle = std::atan2(total_force_y, total_force_x);

      tf2::Quaternion q_base_to_costmap;
      tf2::fromMsg(transform.transform.rotation, q_base_to_costmap);
      const double base_yaw = tf2::getYaw(q_base_to_costmap);

      return global_angle - base_yaw;
    }
    catch (const tf2::TransformException &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "TF lookup failed: %s", e.what());
      return std::nullopt;
    }
  }

  EscapeAction::EscapeAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
  {
    RCLCPP_INFO(node_->get_logger(), "Initializing EscapeAction");
    // 初始化TF相关
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    getInput("robot_frame", robot_frame_);
    getInput("backup_dist", backup_dist_);
    getInput("backup_speed", speed);
    getInput("time_allowance", time_allowance);
    getInput("looking_radius", radius);
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    costmap_sub_on = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
          RCLCPP_INFO(node_->get_logger(), "Success: Costmap received");
          std::lock_guard<std::mutex> lock(costmap_mutex_);
          current_costmap_ = msg; // 更新 costmap
        },
        sub_option);
    if (!costmap_sub_on)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create costmap subscription");
    }

    RCLCPP_INFO(node_->get_logger(), "Starting receive ");
  }
  EscapeAction::~EscapeAction()
  {
    RCLCPP_INFO(node_->get_logger(), "Stop EscapeAction");
    costmap_sub_on.reset();
  }
  void EscapeAction::on_tick()
  {

    callback_group_executor_.spin_some();
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap = current_costmap_;
    }
    goal_.target.x = -backup_dist_;
    goal_.target.y = 0;
    if (!costmap)
    {
      RCLCPP_INFO(node_->get_logger(), "Failed : No costmap available , using backup direction");
    }
    else
    {
      // Calculate escape direction
      auto angle = this->calculate_escape_angle(*costmap, radius, robot_frame_);
      if (!angle)
      {
        RCLCPP_INFO(node_->get_logger(), "Failed to calculate escape direction , using backup direction");
      }
      else
      {
        goal_.target.x = backup_dist_ * std::cos(*angle);
        goal_.target.y = backup_dist_ * std::sin(*angle);
      }
    }
    goal_.target.z = 0.0;
    goal_.speed = speed;
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    RCLCPP_INFO(node_->get_logger(), "Publishing goal: x = %.2f, y = %.2f, z = %.2f",
                goal_.target.x, goal_.target.y, goal_.target.z);
    increment_recovery_count();
  }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<nav2_behavior_tree::EscapeAction>(
        name, "escape", config);
  };

  factory.registerBuilder<nav2_behavior_tree::EscapeAction>("Escape", builder);
}
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

  /**
   * @brief
   * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
   *       It will re-initialize when halted.
   */
  class EscapeAction : public BtActionNode<nav2_msgs::action::BackUp>
  {

  public:
    /**
     * @brief A constructor for nav2_behavior_tree::EscapeAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    EscapeAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf);

    ~EscapeAction();

    void on_tick() override;
    // void on_wait_for_result() override;

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {
              BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
              BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup"),
              BT::InputPort<double>("time_allowance", 10.0, "Allowed time for reversing"),
              BT::InputPort<double>("looking_radius", 0.5, "Planning radius in meters"),
              BT::InputPort<std::string>("robot_frame", "    ", "your robot frame"),

          });
    }

  private:
    std::optional<double> calculate_escape_angle(
        const nav_msgs::msg::OccupancyGrid &costmap,
        double radius,
        const std::string &robot_frame);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_on;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
    std::mutex costmap_mutex_;
    double backup_dist_;
    double speed;
    double radius;
    double time_allowance;
    std::string robot_frame_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_
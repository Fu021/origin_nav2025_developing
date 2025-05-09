// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
  using EscapeAction = nav2_msgs::action::BackUp;
  class Escape : public TimedBehavior<nav2_msgs::action::BackUp>
  {
  public:
    Escape()
        : TimedBehavior<nav2_msgs::action::BackUp>(),
          feedback_(std::make_shared<typename nav2_msgs::action::BackUp::Feedback>()),
          command_x_(0.0),
          command_y_(0.0),
          command_speed(0.0),
          simulate_ahead_time_(0.0)
    {
    }
    ~Escape() = default;
    Status onRun(const std::shared_ptr<const EscapeAction::Goal> command) override;
    Status onCycleUpdate() override;

  protected:
    /**
     * @brief Check if pose is collision free
     * @param distance Distance to check forward
     * @param cmd_vel current commanded velocity
     * @param pose2d Current pose
     * @return is collision free or not
     */
    bool isCollisionFree(
        const double &distance,
        geometry_msgs::msg::Twist *cmd_vel,
        geometry_msgs::msg::Pose2D &pose2d);

    /**
     * @brief Configuration of behavior action
     */
    void onConfigure() override
    {
      auto node = this->node_.lock();
      if (!node)
      {
        throw std::runtime_error{"Failed to lock node"};
      }

      nav2_util::declare_parameter_if_not_declared(
          node,
          "simulate_ahead_time", rclcpp::ParameterValue(2.0));
      node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
    }

    typename nav2_msgs::action::BackUp::Feedback::SharedPtr feedback_;

    geometry_msgs::msg::PoseStamped initial_pose_;
    double command_x_;
    double command_y_;

    double command_speed;

    rclcpp::Duration command_time_allowance_{0, 0};
    rclcpp::Time end_time_;
    double simulate_ahead_time_;
  };

} // namespace nav2_behaviors

#endif // NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_
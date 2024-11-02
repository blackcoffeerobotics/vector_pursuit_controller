// Copyright (c) 2022 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "vector_pursuit_controller/parameter_handler.hpp"

namespace vector_pursuit_controller
{

ParameterHandler::ParameterHandler(std::string name, ros::NodeHandle& pnh)
{
  pnh.param("odom_topic", params_.odom_topic_, std::string("odom"));
  pnh.param("running_topic", params_.running_topic_, std::string("running"));
  pnh.param("control_frequency", params_.control_frequency_, 20.0);
  pnh.param("k", params_.k_, 8.0);
  pnh.param("desired_linear_vel", params_.desired_linear_vel_, 0.5);
  pnh.param("desired_linear_vel_backwards", params_.desired_linear_vel_backwards_, 0.3);
  pnh.param("min_turning_radius", params_.min_turning_radius_, 1.0);
  pnh.param("lookahead_dist", params_.lookahead_dist_, 0.6);
  pnh.param("min_approach_linear_velocity", params_.min_approach_linear_velocity_, 0.05);
  pnh.param("approach_velocity_scaling_dist", params_.approach_velocity_scaling_dist_, 1.0);
  pnh.param("min_lookahead_dist", params_.min_lookahead_dist_, 0.3);
  pnh.param("max_lookahead_dist", params_.max_lookahead_dist_, 0.9);
  pnh.param("lookahead_time", params_.lookahead_time_, 1.5);
  pnh.param("transform_tolerance", params_.transform_tolerance_, 0.1);
  pnh.param("use_velocity_scaled_lookahead_dist", params_.use_velocity_scaled_lookahead_dist_, false);
  pnh.param("min_linear_velocity", params_.min_linear_velocity_, 0.05);
  pnh.param("max_allowed_time_to_collision_up_to_target", params_.max_allowed_time_to_collision_up_to_target_, 1.0);
  pnh.param("scale_factor", params_.scale_factor_, 0.8);
  pnh.param("use_collision_detection", params_.use_collision_detection_, true);
  pnh.param("use_cost_regulated_linear_velocity_scaling", params_.use_cost_regulated_linear_velocity_scaling_, true);
  pnh.param("allow_reversing", params_.allow_reversing_, true);
  pnh.param("cost_scaling_dist", params_.cost_scaling_dist_, 0.6);
  pnh.param("cost_scaling_gain", params_.cost_scaling_gain_, 1.0);
  pnh.param("inflation_cost_scaling_factor", params_.inflation_cost_scaling_factor_, 3.0);
  pnh.param("use_rotate_to_heading", params_.use_rotate_to_heading_, true);
  pnh.param("rotate_to_heading_min_angle", params_.rotate_to_heading_min_angle_, 0.785);
  pnh.param("path_angular_vel_scaling_angle", params_.path_angular_vel_scaling_angle_, 30.0);
  pnh.param("path_angle_scaling_factor", params_.path_angle_scaling_factor_, 1.2);
  pnh.param("rotate_to_path_max_angular_vel", params_.rotate_to_path_max_angular_vel_, 0.5);
  pnh.param("rotate_to_path_min_angular_vel", params_.rotate_to_path_min_angular_vel_, 0.05);
  pnh.param("max_angular_accel", params_.max_angular_accel_, 3.2);
  pnh.param("max_linear_accel", params_.max_linear_accel_, 2.0);
  pnh.param("max_lateral_accel", params_.max_lateral_accel_, 0.5);
  pnh.param("max_robot_pose_search_dist", params_.max_robot_pose_search_dist_, 10.0);
  pnh.param("use_interpolation", params_.use_interpolation_, true);
  pnh.param("use_heading_from_path", params_.use_heading_from_path_, true);
  pnh.param("goal_angular_vel_scaling_angle", params_.goal_angular_vel_scaling_angle_, 30.0);
  pnh.param("goal_angle_scaling_factor", params_.goal_angle_scaling_factor_, 1.2);
  pnh.param("rotate_to_goal_max_angular_vel", params_.rotate_to_goal_max_angular_vel_, 0.5);
  pnh.param("rotate_to_goal_min_angular_vel", params_.rotate_to_goal_min_angular_vel_, 0.05);
  pnh.param("xy_goal_tolerance", params_.xy_goal_tolerance_, 0.1);
  pnh.param("yaw_tolerance", params_.yaw_tolerance_, 0.02);

  if (params_.inflation_cost_scaling_factor_ <= 0.0) {
    ROS_WARN(
      "%s: The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.", name.c_str());
    params_.use_cost_regulated_linear_velocity_scaling_ = false;
  }

  if (params_.min_turning_radius_ == 0.0) {
    ROS_INFO(
      "%s: Min turning radius is set to 0.0, setting use rotate to heading to true.", name.c_str());
    params_.use_rotate_to_heading_ = true;
  }

  if (params_.use_rotate_to_heading_ && params_.allow_reversing_) {
    ROS_WARN(
      "%s: Both use_rotate_to_heading and allow_reversing are set to true. "
      "Reversing will be overriden in all cases.", name.c_str());
  }
}

ParameterHandler::~ParameterHandler()
{
}

}  // namespace vector_pursuit_controller

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

#ifndef VECTOR_PURSUIT_CONTROLLER__CONTROLLER__PARAMETER_HANDLER_HPP_
#define VECTOR_PURSUIT_CONTROLLER__CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>

namespace vector_pursuit_controller
{

struct Parameters
{
    std::string odom_topic_;
    std::string running_topic_;
    double control_frequency_;
    double k_;
    double desired_linear_vel_;
    double desired_linear_vel_backwards_;
    double lookahead_dist_;
    double max_lookahead_dist_;
    double min_lookahead_dist_;
    double lookahead_time_;
    bool use_velocity_scaled_lookahead_dist_;
    double transform_tolerance_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double min_turning_radius_;
    double min_linear_velocity_;
    double max_lateral_accel_;
    double max_allowed_time_to_collision_up_to_target_;
    double scale_factor_;
    bool use_collision_detection_;
    bool use_cost_regulated_linear_velocity_scaling_;
    double cost_scaling_dist_;
    double cost_scaling_gain_;
    double inflation_cost_scaling_factor_;
    bool use_rotate_to_heading_;
    double rotate_to_heading_min_angle_;
    double path_angular_vel_scaling_angle_;
    double path_angle_scaling_factor_;
    double rotate_to_path_max_angular_vel_;
    double rotate_to_path_min_angular_vel_;
    double max_angular_accel_;
    double max_linear_accel_;
    double goal_dist_tol_;
    double max_robot_pose_search_dist_;
    bool use_interpolation_;
    bool allow_reversing_;
    bool is_reversing_;
    bool use_heading_from_path_;
    double goal_angular_vel_scaling_angle_;
    double goal_angle_scaling_factor_;
    double rotate_to_goal_max_angular_vel_;
    double rotate_to_goal_min_angular_vel_;
    double xy_goal_tolerance_;
    double yaw_tolerance_;
};

/**
 * @class vector_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for vector_pursuit_controller::ParameterHandler
   */
  ParameterHandler(std::string name, ros::NodeHandle& pnh);

  /**
   * @brief Destrructor for vector_pursuit_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  std::mutex mutex_;
  Parameters params_;
};

}  // namespace vector_pursuit_controller

#endif  // VECTOR_PURSUIT_CONTROLLER__CONTROLLER__PARAMETER_HANDLER_HPP_

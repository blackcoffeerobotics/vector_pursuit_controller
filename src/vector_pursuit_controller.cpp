// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2024 Black Coffee Robotics
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
#include <cmath>

#include "vector_pursuit_controller/vector_pursuit_controller.hpp"
#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using rcl_interfaces::msg::ParameterType;

namespace vector_pursuit_controller
{
void VectorPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".k", rclcpp::ParameterValue(8.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_turning_radius", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision_up_to_target",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_accel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_heading_from_path",
    rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".k", k_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".min_turning_radius", min_turning_radius_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_linear_velocity",
    min_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_target",
    max_allowed_time_to_collision_up_to_target_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_lateral_accel", max_lateral_accel_);
  node->get_parameter("controller_frequency", control_frequency);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);
  node->get_parameter(
    plugin_name_ + ".use_heading_from_path",
    use_heading_from_path_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  if (min_turning_radius_ == 0.0) {
    RCLCPP_INFO(
      logger_, "Min turning radius is set to 0.0, setting use rotate to heading to true.");
    use_rotate_to_heading_ = true;
  }

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Both use_rotate_to_heading and allow_reversing are set to true. "
      "Reversing will be overriden in all cases.");
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead_point", 1);
  target_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void VectorPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " vector_pursuit_controller::VectorPursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  target_pub_.reset();
  target_arc_pub_.reset();
}

void VectorPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "vector_pursuit_controller::VectorPursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  target_pub_->on_activate();
  target_arc_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &VectorPursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void VectorPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "vector_pursuit_controller::VectorPursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  target_pub_->on_deactivate();
  target_arc_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

double VectorPursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  // TODO(arthur_bcr): verify that the speed variable is correctly set
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = std::abs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double VectorPursuitController::calcTurningRadius(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  // Calculate angle to lookahead point
  double target_angle = angles::normalize_angle(tf2::getYaw(target_pose.pose.orientation));
  double distance = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
  // Compute turning radius (screw center)
  double turning_radius;
  if (allow_reversing_ || target_pose.pose.position.x >= 0.0) {
    if (std::abs(target_pose.pose.position.y) > 1e-6) {
      double phi_1 = std::atan2(
        (2 * std::pow(target_pose.pose.position.y, 2) - std::pow(distance, 2)),
        (2 * target_pose.pose.position.x * target_pose.pose.position.y));
      double phi_2 = std::atan2(std::pow(distance, 2), (2 * target_pose.pose.position.y));
      double phi = angles::normalize_angle(phi_1 - phi_2);
      double term_1 = (k_ * phi) / (((k_ - 1) * phi) + target_angle);
      double term_2 = std::pow(distance, 2) / (2 * target_pose.pose.position.y);
      turning_radius = std::abs(term_1 * term_2);
    } else {
      // Handle case when target is directly ahead
      turning_radius = std::numeric_limits<double>::max();
    }
  } else {
    // If lookahead point is behind the robot, set turning radius to minimum
    turning_radius = min_turning_radius_;
  }

  // Limit turning radius to avoid extremely sharp turns
  turning_radius = std::max(turning_radius, min_turning_radius_);
  RCLCPP_DEBUG(logger_, "Turning radius: %f", turning_radius);

  return turning_radius;
}

geometry_msgs::msg::TwistStamped VectorPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update goal tolerances
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path
  double lookahead_dist = getLookAheadDistance(last_cmd_vel_);

  // Cusp check
  const double dist_to_cusp = getCuspDist(transformed_plan);

  // if the lookahead distance is further than the cusp, use the cusp distance instead
  if (dist_to_cusp < lookahead_dist) {
    lookahead_dist = dist_to_cusp;
  }

  auto lookahead_point = getLookAheadPoint(lookahead_dist, transformed_plan);

  // Publish target point for visualization
  target_pub_->publish(lookahead_point);

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = lookahead_point.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  double linear_vel, angular_vel;

  linear_vel = desired_linear_vel_;

  double angle_to_heading;
  if (shouldRotateToGoalHeading(lookahead_point)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, last_cmd_vel_);
  } else if (shouldRotateToPath(lookahead_point, angle_to_heading, sign)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, last_cmd_vel_);
  } else {
    double turning_radius = calcTurningRadius(lookahead_point);

    // Compute linear velocity based on path curvature
    double curvature = 1.0 / turning_radius;

    applyConstraints(
      curvature, last_cmd_vel_,
      costAtPose(pose.pose.position.x, pose.pose.position.y), linear_vel, transformed_plan, sign);

    // Compute angular velocity
    angular_vel = linear_vel / turning_radius;
    if (lookahead_point.pose.position.y < 0) {
      angular_vel *= -1;
    }
  }

  // Collision checking
  const double target_dist = std::hypot(
    lookahead_point.pose.position.x,
    lookahead_point.pose.position.y);

  if (use_collision_detection_ &&
    isCollisionImminent(pose, linear_vel, angular_vel, target_dist))
  {
    linear_vel = 0.0;
    angular_vel = 0.0;
    throw nav2_core::PlannerException("VectorPursuitController: Collision detected ahead!");
  }

  // Populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  // TODO(kostubh_bcr): BUG in recieved speed param from
  // controller server (in binaries 1.1.15).
  // Use speed instead when branch with the fix is merged.
  last_cmd_vel_ = speed;
  last_cmd_vel_ = cmd_vel.twist;
  return cmd_vel;
}

double VectorPursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");

    throw nav2_core::PlannerException(
            "VectorPursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

double VectorPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void VectorPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = desired_linear_vel_;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel = unbounded_vel;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void VectorPursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
  const double & pose_cost, double & linear_vel, const nav_msgs::msg::Path & path, double & sign)
{
  double cost_vel = linear_vel;

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
    pose_cost != static_cast<double>(nav2_costmap_2d::FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // limit the linear velocity by curvature
  double max_vel_for_curve = std::sqrt(max_lateral_accel_ / std::abs(curvature));

  // Apply constraints
  linear_vel = std::min(
    {linear_vel, max_vel_for_curve, cost_vel,
      std::abs(curr_speed.linear.x) + max_linear_accel_ * control_duration_});

  applyApproachVelocityScaling(path, linear_vel);

  // Ensure the linear velocity is not below the minimum allowed linear velocity
  linear_vel = std::max(linear_vel, min_linear_velocity_);
  linear_vel = sign * linear_vel;
}

bool VectorPursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & target_pose,
  double & angle_to_path, double & sign)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(target_pose.pose.position.y, target_pose.pose.position.x);

  // In case we are reversing
  if (sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }

  return use_rotate_to_heading_ && std::abs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool VectorPursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

geometry_msgs::msg::Point VectorPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::Quaternion VectorPursuitController::getOrientation(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  tf2::Quaternion tf2_quat;

  double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  tf2_quat.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(tf2_quat);

  return quat_msg;
}

double getPositiveRadians(double angle)
{
  // check for infinity or NaN
  if (isnan(angle) || isinf(angle)) {return 0.0;}

  while (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::PoseStamped VectorPursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  geometry_msgs::msg::PoseStamped pose;  // pose to return

  // If no pose is far enough, take the last pose discretely
  if (goal_pose_it == transformed_plan.poses.end()) {
    pose = *(std::prev(transformed_plan.poses.end()));  // dereference the last element pointer

    // if heading needs to be computed from path,
    // find the angle of the vector from second last to last pose
    if (!use_heading_from_path_) {
      pose.pose.orientation = getOrientation(
        std::prev(std::prev(transformed_plan.poses.end()))->pose.position,
        std::prev(transformed_plan.poses.end())->pose.position);
    }

    // if the first pose is ahead of the lookahead distance, take the first pose discretely
  } else if (goal_pose_it == transformed_plan.poses.begin()) {
    pose = *(goal_pose_it);  // dereference the first element pointer

    // if heading needs to be computed from path,
    // find the angle of the vector from first to second pose
    if (!use_heading_from_path_) {
      pose.pose.orientation = getOrientation(
        transformed_plan.poses.begin()->pose.position,
        std::next(transformed_plan.poses.begin())->pose.position);
    }

    // if interpolation is enabled:
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be
    // inside the circle, and goal_pose is guaranteed to be outside the circle.
  } else if (use_interpolation_) {
    auto prev_pose_it = std::prev(goal_pose_it);

    pose.pose.position = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);

    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;

    // if heading needs to be computed from path,
    // find the angle of the vector from prev to goal pose
    if (!use_heading_from_path_) {
      pose.pose.orientation = getOrientation(
        prev_pose_it->pose.position, goal_pose_it->pose.position);

      // use the headings from the prev and goal poses to interpolate
      // a new heading for the lookahead point
    } else {
      double goal_yaw = getPositiveRadians(tf2::getYaw(goal_pose_it->pose.orientation));
      double prev_yaw = getPositiveRadians(tf2::getYaw(prev_pose_it->pose.orientation));
      double interpolated_yaw = angles::normalize_angle((goal_yaw + prev_yaw) / 2.0);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sin(interpolated_yaw / 2.0);
      pose.pose.orientation.w = cos(interpolated_yaw / 2.0);
    }

    // if interpolation is disabled just return selected goal pose
  } else {
    pose = *(goal_pose_it);

    // if heading needs to be computed from path,
    // find the angle of the vector from second last to last pose
    if (!use_heading_from_path_) {
      pose.pose.orientation = getOrientation(
        std::prev(goal_pose_it)->pose.position, goal_pose_it->pose.position);
    }
  }
  return pose;
}

void VectorPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void VectorPursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

bool VectorPursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & target_dist)
{
  // This may be a bit unusual, but the robot_pose is in
  // odom frame and the target_pose is in robot base frame.

  // check current point is OK
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    RCLCPP_WARN(logger_, "Robot is in collision at current pose");
    return true;
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  double projection_time = 0.0;
  if (std::abs(linear_vel) < 0.01 && std::abs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time =
      2.0 * sin((costmap_->getResolution() / 2) / max_radius) / std::abs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = costmap_->getResolution() / std::abs(linear_vel);
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  // only forward simulate within time requested
  int i = 1;
  while (i * projection_time < max_allowed_time_to_collision_up_to_target_) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // check if past target pose, where no longer a thoughtfully valid command
    if (std::hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > target_dist) {
      break;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      target_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  target_arc_pub_->publish(arc_pts_msg);

  return false;
}

double VectorPursuitController::getCuspDist(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_prod < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }

    if (
      (hypot(oa_x, oa_y) == 0.0 &&
      transformed_plan.poses[pose_id - 1].pose.orientation !=
      transformed_plan.poses[pose_id].pose.orientation)
      ||
      (hypot(ab_x, ab_y) == 0.0 &&
      transformed_plan.poses[pose_id].pose.orientation !=
      transformed_plan.poses[pose_id + 1].pose.orientation))
    {
      // returning the distance since the points overlap
      // but are not simply duplicate points (e.g. in place rotation)
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool VectorPursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());
  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    RCLCPP_WARN(logger_, "Footprint cost is unknown, collision check failed");
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

void VectorPursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path VectorPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool VectorPursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double VectorPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}


rcl_interfaces::msg::SetParametersResult VectorPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          continue;
        }
        inflation_cost_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_linear_vel") {
        desired_linear_vel_ = parameter.as_double();
        base_desired_linear_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_linear_velocity") {
        min_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_target") {
        max_allowed_time_to_collision_up_to_target_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_dist") {
        cost_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_gain") {
        cost_scaling_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_min_angle") {
        rotate_to_heading_min_angle_ = parameter.as_double();
      } else if (name == plugin_name_ + ".k") {
        k_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_turning_radius") {
        min_turning_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lateral_accel") {
        max_lateral_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_linear_accel") {
        max_linear_accel_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".allow_reversing") {
        allow_reversing_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_collision_detection") {
        use_collision_detection_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_heading_from_path") {
        use_heading_from_path_ = parameter.as_bool();
      }
    }
  }

  if (min_turning_radius_ == 0.0) {
    RCLCPP_INFO(
      logger_, "Min turning radius is set to 0.0, setting use rotate to heading to true.");
    use_rotate_to_heading_ = true;
  }

  result.successful = true;
  return result;
}

}  // namespace vector_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  vector_pursuit_controller::VectorPursuitController,
  nav2_core::Controller)

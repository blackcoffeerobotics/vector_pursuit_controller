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

#include <vector_pursuit_controller/vector_pursuit_controller.hpp>
#include <angles/angles.h>
#include <vector_pursuit_controller/utils.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/TwistStamped.h>

// Register this controller as a nav_core plugin
PLUGINLIB_EXPORT_CLASS(
  vector_pursuit_controller::VectorPursuitController,
  nav_core::BaseLocalPlanner)

using vector_pursuit_controller::euclidean_distance;
using vector_pursuit_controller::orientationAroundZAxis;
using vector_pursuit_controller::first_after_integrated_distance;
using vector_pursuit_controller::min_by;
using vector_pursuit_controller::calculate_path_length;
using vector_pursuit_controller::transformPose;
using vector_pursuit_controller::clamp;

namespace vector_pursuit_controller
{

VectorPursuitController::VectorPursuitController()
:   initialized_(false),
    is_running_(false),
    goal_reached_(false),
    check_xy_(true)
{
}

VectorPursuitController::~VectorPursuitController()
{
    if (dynamic_srv_) {
        delete dynamic_srv_;
        dynamic_srv_ = nullptr;  // Optional: Helps to avoid dangling pointer issues.
    }
}

void VectorPursuitController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_) {
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_("~" + name);
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        // Handles storage and dynamic configuration of parameters.
        // Returns pointer to data current param settings.
        param_handler_ = std::make_unique<ParameterHandler>(controller_name_, pnh_);
        params_ = param_handler_->getParams();

        control_duration_ = 1.0 / params_->control_frequency_;

        // init the odom helper to receive the robot's velocity from odom messages
        odom_helper_.setOdomTopic(params_->odom_topic_);

        global_path_pub_ = nh_.advertise<nav_msgs::Path>("received_global_plan", 1);
        target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lookahead_point", 1);
        target_arc_pub_ = nh_.advertise<nav_msgs::Path>("lookahead_collision_arc", 1);

        running_sub_ = nh_.subscribe<std_msgs::Bool>(params_->running_topic_, 5,
                                    &VectorPursuitController::runningCallback, this);

        // initialize collision checker and set costmap
        collision_checker_ = std::make_unique<FootprintCollisionChecker<costmap_2d::Costmap2D *>>(costmap_);
        collision_checker_->setCostmap(costmap_);

        // Set up parameter reconfigure
        dynamic_srv_ = new ParamterConfigServer(pnh_);
        CallbackType cb = boost::bind(&VectorPursuitController::reconfigureCB, this, _1, _2);
        dynamic_srv_->setCallback(cb);

        ROS_INFO("Created %s plugin.", controller_name_.c_str());
        initialized_ = true;
    }
}

void VectorPursuitController::runningCallback(
    const std_msgs::Bool::ConstPtr &msg)
{
    is_running_ = msg->data;
}

void VectorPursuitController::reconfigureCB(Config& config, uint32_t level)
{
  if (!initialized_) {
    return;
  }
  ROS_INFO("%s: Got e new reconfigure.", controller_name_.c_str());
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  params_->control_frequency_ = config.controller_frequency;
  control_duration_ = 1.0 / params_->control_frequency_;
  params_->k_ = config.k;
  params_->desired_linear_vel_ = config.desired_linear_vel;
  params_->desired_linear_vel_backwards_ = config.desired_linear_vel_backwards;
  params_->min_turning_radius_ = config.min_turning_radius;
  params_->lookahead_dist_ = config.lookahead_dist;
  params_->min_approach_linear_velocity_ = config.min_approach_linear_velocity;
  params_->approach_velocity_scaling_dist_ = config.approach_velocity_scaling_dist;
  params_->min_lookahead_dist_ = config.min_lookahead_dist;
  params_->max_lookahead_dist_ = config.max_lookahead_dist;
  params_->lookahead_time_ = config.lookahead_time;
  params_->use_velocity_scaled_lookahead_dist_ = config.use_velocity_scaled_lookahead_dist;
  params_->min_linear_velocity_ = config.min_linear_velocity;
  params_->max_allowed_time_to_collision_up_to_target_ = config.max_allowed_time_to_collision_up_to_target;
  params_->scale_factor_ = config.scale_factor;
  params_->use_collision_detection_ = config.use_collision_detection;
  params_->use_cost_regulated_linear_velocity_scaling_ = config.use_cost_regulated_linear_velocity_scaling;
  params_->allow_reversing_ = config.allow_reversing;
  params_->cost_scaling_dist_ = config.cost_scaling_dist;
  params_->cost_scaling_gain_ = config.cost_scaling_gain;
  params_->inflation_cost_scaling_factor_ = config.inflation_cost_scaling_factor;
  params_->max_angular_accel_ = config.max_angular_accel;
  params_->max_linear_accel_ = config.max_linear_accel;
  params_->max_lateral_accel_ = config.max_lateral_accel;
  params_->max_robot_pose_search_dist_ = config.max_robot_pose_search_dist;
  params_->use_interpolation_ = config.use_interpolation;
  params_->use_heading_from_path_ = config.use_heading_from_path;
  params_->goal_angular_vel_scaling_angle_ = config.goal_angular_vel_scaling_angle;
  params_->goal_angle_scaling_factor_ = config.goal_angle_scaling_factor;
  params_->rotate_to_goal_max_angular_vel_ = config.rotate_to_goal_max_angular_vel;
  params_->rotate_to_goal_min_angular_vel_ = config.rotate_to_goal_min_angular_vel;
  params_->xy_goal_tolerance_ = config.xy_goal_tolerance;
  params_->yaw_tolerance_ = config.yaw_tolerance;
}

double VectorPursuitController::getLookAheadDistance(
    const geometry_msgs::Twist & speed)
{
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    // TODO(arthur_bcr): verify that the speed variable is correctly set
    double lookahead_dist = params_->lookahead_dist_;
    if (params_->use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = std::abs(speed.linear.x) * params_->lookahead_time_;
    lookahead_dist = clamp(lookahead_dist, params_->min_lookahead_dist_, params_->max_lookahead_dist_);
    }

    return lookahead_dist;
}

double VectorPursuitController::calcTurningRadius(
    const geometry_msgs::PoseStamped & target_pose)
{
    // Calculate angle to lookahead point
    double target_angle = angles::normalize_angle(tf2::getYaw(target_pose.pose.orientation));
    double distance = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
    // Compute turning radius (screw center)
    double turning_radius;
    if (params_->allow_reversing_ || target_pose.pose.position.x >= 0.0) {
        if (std::abs(target_pose.pose.position.y) > 1e-6) {
        double phi_1 = std::atan2(
            (2 * std::pow(target_pose.pose.position.y, 2) - std::pow(distance, 2)),
            (2 * target_pose.pose.position.x * target_pose.pose.position.y));
        double phi_2 = std::atan2(std::pow(distance, 2), (2 * target_pose.pose.position.y));
        double phi = angles::normalize_angle(phi_1 - phi_2);
        double term_1 = (params_->k_ * phi) / (((params_->k_ - 1) * phi) + target_angle);
        double term_2 = std::pow(distance, 2) / (2 * target_pose.pose.position.y);
        turning_radius = std::abs(term_1 * term_2);
        } else {
        // Handle case when target is directly ahead
        turning_radius = std::numeric_limits<double>::max();
        }
    } else {
        // If lookahead point is behind the robot, set turning radius to minimum
        turning_radius = params_->min_turning_radius_;
    }

    // Limit turning radius to avoid extremely sharp turns
    turning_radius = std::max(turning_radius, params_->min_turning_radius_);
    ROS_DEBUG("%s: Turning radius: %f", controller_name_.c_str(), turning_radius);

    return turning_radius;
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

bool VectorPursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    // check if plugin is initialized
    if(!initialized_)
    {
        ROS_ERROR("%s has not been initialized, please call initialize() before using this planner",
                  controller_name_.c_str());
        return false;
    }

    // store the global plan
    global_plan_.header.stamp = ros::Time::now();
    global_plan_.header.frame_id = orig_global_plan.back().header.frame_id;
    global_plan_.poses = orig_global_plan;

    goal_.header.frame_id = global_plan_.header.frame_id;
    goal_.pose = orig_global_plan.back().pose;

    goal_reached_ = false;
                    
    return true;
}

bool VectorPursuitController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::lock_guard<std::mutex> lock_reinit(mutex_);
    costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // robot_pose (base_robot_frame - odom_frame)
    // curr_robot_pose (base_robot_frame - map_frame)
    geometry_msgs::PoseStamped robot_pose, curr_robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    goal_reached_ = false;
    nav_msgs::Path transformed_plan;

    // Transform path to robot base frame
    try {
        if (!transformGlobalPlan(robot_pose, transformed_plan)) {
            return false;
        }
    }
    catch(const std::exception& e) {
        return false;
    }

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
    target_pub_.publish(lookahead_point);

    // Setting the velocity direction
    double linear_vel, angular_vel;
    double sign = 1.0;
    if (params_->allow_reversing_) {
        sign = lookahead_point.pose.position.x >= 0.0 ? 1.0 : -1.0;
        if (lookahead_point.pose.position.x >= 0.0) {
            sign = 1.0;
            linear_vel = params_->desired_linear_vel_;
        } else {
            sign = -1.0;
            linear_vel = params_->desired_linear_vel_backwards_;
        }
    } else {
        linear_vel = params_->desired_linear_vel_;
    }

    double angle_to_heading;
    double angle_to_goal;
    
    // Check if xy reached
    if (isGoalReached(robot_pose, angle_to_goal)) {
        if (shouldRotateToGoalHeading(angle_to_goal)) {
            ROS_DEBUG("%s: Rotating to GOAL heading...", controller_name_.c_str());
            rotateToGoal(linear_vel, angular_vel, angle_to_goal);
        }
        else {
            goal_reached_ = true;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            last_cmd_vel_ = cmd_vel;
            return true;
        }
    } else if (shouldRotateToPath(lookahead_point, angle_to_heading, sign)) {
        rotateToHeading(linear_vel, angular_vel, angle_to_heading);
        ROS_DEBUG("%s: Rotating to PATH heading...", controller_name_.c_str());
    }
    else {
        double turning_radius = calcTurningRadius(lookahead_point);

        // Compute linear velocity based on path curvature
        double curvature = 1.0 / turning_radius;

        double cost;
        if (!costAtPose(robot_pose.pose.position.x, robot_pose.pose.position.y, cost)) {
            return false;
        }

        applyConstraints(
        curvature, last_cmd_vel_, cost, linear_vel, transformed_plan, sign);

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

    if (params_->use_collision_detection_ &&
        isCollisionImminent(robot_pose, linear_vel, angular_vel, target_dist))
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        last_cmd_vel_ = cmd_vel;
        ROS_ERROR_THROTTLE(3, "%s: Collision detected ahead!", controller_name_.c_str());
        return false;
    }

    // Populate and return message
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;

    // TODO(kostubh_bcr): BUG in recieved speed param from
    // controller server (in binaries 1.1.15).
    // Use speed instead when branch with the fix is merged.
    last_cmd_vel_ = cmd_vel;
    return true;
}

bool VectorPursuitController::isGoalReached()
{
    if (goal_reached_)
    {
        check_xy_ = true;
        ROS_INFO("%s: GOAL Reached!", controller_name_.c_str());
        return true;
    }
    return false;
}

bool VectorPursuitController::isGoalReached(
    geometry_msgs::PoseStamped robot_pose,
    double &dyaw)
{
    geometry_msgs::PoseStamped transformed_end_pose;
    if (!transformPose(tf_, costmap_ros_->getGlobalFrameID(),
        goal_, transformed_end_pose, params_->transform_tolerance_)) {
        return false;
    }
    
    if (prev_goal_.pose != goal_.pose || is_running_) {
        check_xy_ = true;
        is_running_ = false;
    }
    prev_goal_.pose = goal_.pose;
    
    double dx = robot_pose.pose.position.x - transformed_end_pose.pose.position.x,
           dy = robot_pose.pose.position.y - transformed_end_pose.pose.position.y;
    
    if (check_xy_) {
        if (dx*dx + dy*dy <= params_->xy_goal_tolerance_*params_->xy_goal_tolerance_) {
            check_xy_ = false;
        }
        else {
            return false;
        }
    }
    dyaw = angles::shortest_angular_distance(
           tf2::getYaw(robot_pose.pose.orientation),
           tf2::getYaw(transformed_end_pose.pose.orientation));

    return true;
}

bool VectorPursuitController::transformGlobalPlan(
    const geometry_msgs::PoseStamped & pose,
    nav_msgs::Path& transformed_plan)
{
    if (global_plan_.poses.empty()) {
        ROS_ERROR_THROTTLE(2, "%s: Received plan with zero length", controller_name_.c_str());
        return false;
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    if (!transformPose(tf_,
                       global_plan_.header.frame_id,
                       pose, robot_pose, params_->transform_tolerance_)) {
        ROS_ERROR_THROTTLE(2, "%s: Unable to transform robot pose into global plan's frame", controller_name_.c_str());
        return false;
    }

    // We'll discard points on the plan that are outside the local costmap
    double max_costmap_extent = getCostmapMaxExtent();

    auto closest_pose_upper_bound = first_after_integrated_distance(
        global_plan_.poses.begin(), global_plan_.poses.end(), params_->max_robot_pose_search_dist_);

    // First find the closest pose on the path to the robot
    // bounded by when the path turns around (if it does) so we don't get a pose from a later
    // portion of the path
    auto transformation_begin = min_by(
        global_plan_.poses.begin(), closest_pose_upper_bound,
        [&robot_pose](const geometry_msgs::PoseStamped & ps) {
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
        geometry_msgs::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = robot_pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        if (!transformPose(tf_,
                           costmap_ros_->getBaseFrameID(),
                           stamped_pose,
                           transformed_pose, params_->transform_tolerance_)) {
            throw std::runtime_error("VectorPursuitController: "
                    "Unable to transform from '" + costmap_ros_->getBaseFrameID()
                    + "' to '" + stamped_pose.header.frame_id + "'!");
        }
        transformed_pose.pose.position.z = 0.0;
        return transformed_pose;
        };

    // Transform the near part of the global plan into the robot's frame of reference.
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_path_pub_.publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        ROS_ERROR_THROTTLE(2, "%s: Resulting plan has 0 poses in it.", controller_name_.c_str());
        return false;
    }
    return true;
}

double VectorPursuitController::getCostmapMaxExtent() const
{
    const double max_costmap_dim_meters = std::max(
        costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0;
}

double VectorPursuitController::getCuspDist(
  const nav_msgs::Path & transformed_plan)
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
        return std::hypot(
            transformed_plan.poses[pose_id].pose.position.x,
            transformed_plan.poses[pose_id].pose.position.y);
        }

        if (
        (std::hypot(oa_x, oa_y) == 0.0 &&
        transformed_plan.poses[pose_id - 1].pose.orientation !=
        transformed_plan.poses[pose_id].pose.orientation)
        ||
        (std::hypot(ab_x, ab_y) == 0.0 &&
        transformed_plan.poses[pose_id].pose.orientation !=
        transformed_plan.poses[pose_id + 1].pose.orientation))
        {
        // returning the distance since the points overlap
        // but are not simply duplicate points (e.g. in place rotation)
        return std::hypot(
            transformed_plan.poses[pose_id].pose.position.x,
            transformed_plan.poses[pose_id].pose.position.y);
        }
    }

    return std::numeric_limits<double>::max();
}

geometry_msgs::PoseStamped VectorPursuitController::getLookAheadPoint(
    const double & lookahead_dist,
    const nav_msgs::Path & transformed_plan)
{
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const geometry_msgs::PoseStamped & ps)
        {
        return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
        });

    geometry_msgs::PoseStamped pose;  // pose to return

    // If no pose is far enough, take the last pose discretely
    if (goal_pose_it == transformed_plan.poses.end()) {
        pose = *(std::prev(transformed_plan.poses.end()));  // dereference the last element pointer

        // if heading needs to be computed from path,
        // find the angle of the vector from second last to last pose
        if (!params_->use_heading_from_path_) {
        pose.pose.orientation = getOrientation(
            std::prev(std::prev(transformed_plan.poses.end()))->pose.position,
            std::prev(transformed_plan.poses.end())->pose.position);
        }

        // if the first pose is ahead of the lookahead distance, take the first pose discretely
    } else if (goal_pose_it == transformed_plan.poses.begin()) {
        pose = *(goal_pose_it);  // dereference the first element pointer

        // if heading needs to be computed from path,
        // find the angle of the vector from first to second pose
        if (!params_->use_heading_from_path_) {
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
    } else if (params_->use_interpolation_) {
        auto prev_pose_it = std::prev(goal_pose_it);

        pose.pose.position = circleSegmentIntersection(
        prev_pose_it->pose.position,
        goal_pose_it->pose.position, lookahead_dist);

        pose.header.frame_id = prev_pose_it->header.frame_id;
        pose.header.stamp = goal_pose_it->header.stamp;

        // if heading needs to be computed from path,
        // find the angle of the vector from prev to goal pose
        if (!params_->use_heading_from_path_) {
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
        if (!params_->use_heading_from_path_) {
        pose.pose.orientation = getOrientation(
            std::prev(goal_pose_it)->pose.position, goal_pose_it->pose.position);
        }
    }
    return pose;
}

bool VectorPursuitController::shouldRotateToPath(
  const geometry_msgs::PoseStamped & target_pose,
  double & angle_to_path, double & sign)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(target_pose.pose.position.y, target_pose.pose.position.x);

  // In case we are reversing
  if (sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }

  return params_->use_rotate_to_heading_ && std::abs(angle_to_path) > params_->rotate_to_heading_min_angle_;
}

bool VectorPursuitController::shouldRotateToGoalHeading(double angle_to_goal)
{
    // Whether we should rotate robot to goal heading
    return fabs(angle_to_goal) >= params_->yaw_tolerance_;
}

void VectorPursuitController::rotateToGoal(
    double & linear_vel,
    double & angular_vel,
    const double & angle_to_goal)
{
    // Rotate in place using max angular velocity / acceleration possible
    linear_vel = 0.0;
    const double sign = angle_to_goal > 0.0 ? 1.0 : -1.0;
    double angle_to_goal_;
        
    if (std::abs(angle_to_goal) < angles::from_degrees(params_->goal_angular_vel_scaling_angle_)) {
        angle_to_goal_ = std::abs(angle_to_goal) / params_->goal_angle_scaling_factor_;
    } else {
        angle_to_goal_ = 1.0;
    }

    double rotate_to_goal_angular_vel = params_->rotate_to_goal_max_angular_vel_;
    double unbounded_angular_vel = rotate_to_goal_angular_vel * angle_to_goal_;

    if (unbounded_angular_vel < params_->rotate_to_goal_min_angular_vel_) {
        rotate_to_goal_angular_vel = params_->rotate_to_goal_min_angular_vel_;
    } else {
        rotate_to_goal_angular_vel = unbounded_angular_vel;
    }
    angular_vel = sign*clamp(rotate_to_goal_angular_vel,
                             params_->rotate_to_goal_min_angular_vel_,
                             params_->rotate_to_goal_max_angular_vel_);
}

void VectorPursuitController::rotateToHeading(
    double & linear_vel,
    double & angular_vel,
    const double & angle_to_path)
{
    // Rotate in place using max angular velocity / acceleration possible
    linear_vel = 0.0;
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    double angle_to_path_;
        
    if (std::abs(angle_to_path) < angles::from_degrees(params_->path_angular_vel_scaling_angle_)) {
        angle_to_path_ = std::abs(angle_to_path) / params_->path_angle_scaling_factor_;
    } else {
        angle_to_path_ = 1.0;
    }

    double rotate_to_path_angular_vel = params_->rotate_to_path_max_angular_vel_;
    double unbounded_angular_vel = rotate_to_path_angular_vel * angle_to_path_;

    if (unbounded_angular_vel < params_->rotate_to_path_min_angular_vel_) {
        rotate_to_path_angular_vel = params_->rotate_to_path_min_angular_vel_;
    } else {
        rotate_to_path_angular_vel = unbounded_angular_vel;
    }
    angular_vel = sign*clamp(rotate_to_path_angular_vel,
                             params_->rotate_to_path_min_angular_vel_,
                             params_->rotate_to_path_max_angular_vel_);
}

bool VectorPursuitController::costAtPose(const double & x, const double & y, double &cost)
{
  unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my)) {
        ROS_FATAL(
        "%s: "
        "The dimensions of the costmap is too small to fully include your robot's footprint, "
        "thusly the robot cannot proceed further", controller_name_.c_str());

        ROS_ERROR("%s: "
                "Dimensions of the costmap are too small "
                "to encapsulate the robot footprint at current speeds!", controller_name_.c_str());
        return false;
    }

    unsigned char cost_ = costmap_->getCost(mx, my);
    cost = static_cast<double>(cost_);
    return true;
}

bool VectorPursuitController::inCollision(
    const double & x,
    const double & y,
    const double & theta)
{
    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my)) {
        ROS_WARN_THROTTLE(30000, "%s: "
        "The dimensions of the costmap is too small to successfully check for "
        "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
        "increase your costmap size.", controller_name_.c_str());
        return false;
    }

    double footprint_cost = collision_checker_->footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint(), params_->scale_factor_);
    if (footprint_cost == static_cast<double>(costmap_2d::NO_INFORMATION) &&
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
        ROS_WARN_THROTTLE(5, "%s: Footprint cost is unknown, collision check failed", controller_name_.c_str());
        return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
}

bool VectorPursuitController::isCollisionImminent(
    const geometry_msgs::PoseStamped & robot_pose,
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
        ROS_WARN("%s: Robot is in collision at current pose", controller_name_.c_str());
        return true;
    }

    // visualization messages
    nav_msgs::Path arc_pts_msg;
    arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
    arc_pts_msg.header.stamp = robot_pose.header.stamp;
    geometry_msgs::PoseStamped pose_msg;
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

    const geometry_msgs::Point & robot_xy = robot_pose.pose.position;
    geometry_msgs::Pose2D curr_pose;
    curr_pose.x = robot_pose.pose.position.x;
    curr_pose.y = robot_pose.pose.position.y;
    curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

    // only forward simulate within time requested
    int i = 1;
    while (i * projection_time < params_->max_allowed_time_to_collision_up_to_target_) {
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
        target_arc_pub_.publish(arc_pts_msg);
        return true;
        }
    }

    target_arc_pub_.publish(arc_pts_msg);

    return false;
}

void VectorPursuitController::applyConstraints(
    const double & curvature, const geometry_msgs::Twist & curr_speed,
    const double & pose_cost, double & linear_vel, const nav_msgs::Path & path, double & sign)
{
    double cost_vel = linear_vel;

    // limit the linear velocity by proximity to obstacles
    if (params_->use_cost_regulated_linear_velocity_scaling_ &&
        pose_cost != static_cast<double>(costmap_2d::NO_INFORMATION) &&
        pose_cost != static_cast<double>(costmap_2d::FREE_SPACE))
    {
        const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
        const double min_distance_to_obstacle = (-1.0 / params_->inflation_cost_scaling_factor_) *
        std::log(pose_cost / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

        if (min_distance_to_obstacle < params_->cost_scaling_dist_) {
        cost_vel *= params_->cost_scaling_gain_ * min_distance_to_obstacle / params_->cost_scaling_dist_;
        }
    }

    // limit the linear velocity by curvature
    double max_vel_for_curve = std::sqrt(params_->max_lateral_accel_ / std::abs(curvature));

    // Apply constraints
    linear_vel = std::min(
        {linear_vel, max_vel_for_curve, cost_vel,
        std::abs(curr_speed.linear.x) + params_->max_linear_accel_ * control_duration_});

    applyApproachVelocityScaling(path, linear_vel);

    // Ensure the linear velocity is not below the minimum allowed linear velocity
    linear_vel = std::max(linear_vel, params_->min_linear_velocity_);
    linear_vel = sign * linear_vel;
}

void VectorPursuitController::applyApproachVelocityScaling(
    const nav_msgs::Path & path,
    double & linear_vel
) const
{
    double approach_vel = params_->desired_linear_vel_;
    double velocity_scaling = approachVelocityScalingFactor(path);
    double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < params_->min_approach_linear_velocity_) {
        approach_vel = params_->min_approach_linear_velocity_;
    } else {
        approach_vel = unbounded_vel;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    linear_vel = std::min(linear_vel, approach_vel);
}

double VectorPursuitController::approachVelocityScalingFactor(
    const nav_msgs::Path & transformed_path) const
{
    // Waiting to apply the threshold based on integrated distance ensures we don't
    // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
    double remaining_distance = calculate_path_length(transformed_path);
    if (remaining_distance < params_->approach_velocity_scaling_dist_) {
        auto & last = transformed_path.poses.back();
        double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
        return distance_to_last_pose / params_->approach_velocity_scaling_dist_;
    } else {
        return 1.0;
    }
}

geometry_msgs::Point VectorPursuitController::circleSegmentIntersection(
    const geometry_msgs::Point & p1,
    const geometry_msgs::Point & p2,
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

    geometry_msgs::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

geometry_msgs::Quaternion VectorPursuitController::getOrientation(
    const geometry_msgs::Point & p1,
    const geometry_msgs::Point & p2)
{
    tf2::Quaternion tf2_quat;

    double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
    tf2_quat.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf2_quat);

    return quat_msg;
}

} // namespace vector_pursuit_controller
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

#ifndef VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
#define VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <tf2_ros/buffer.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <vector_pursuit_controller/footprint_collision_checker.hpp>
#include <vector_pursuit_controller/parameter_handler.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <vector_pursuit_controller/VectorPursuitControllerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace vector_pursuit_controller
{

/**
 * @class nav2_vector_pursuit::VectorPursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class VectorPursuitController : public nav_core::BaseLocalPlanner
{

typedef vector_pursuit_controller::VectorPursuitControllerConfig Config;
typedef dynamic_reconfigure::Server<Config> ParamterConfigServer;
typedef dynamic_reconfigure::Server<Config>::CallbackType CallbackType;

public:
    /**
     * @brief Constructor for nav2_vector_pursuit::VectorPursuitController
     */
    VectorPursuitController();

    /**
     * @brief Destrructor for nav2_vector_pursuit::VectorPursuitController
     */
    ~VectorPursuitController();

    /**
     * @brief Initializes the plugin
     * @param name The name of the instance
     * @param tf Pointer to a tf buffer
     * @param costmap_ros Cost map representing occupied and free space
     */
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief Set the plan that the local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
    * @brief  Check if the goal pose has been achieved
    * 
    * The actual check is performed in computeVelocityCommands(). 
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
    bool isGoalReached();

    /**
     * @brief Check if robot reaches xy
     * @return If true, check yaw -> True if yaw reached.
     */
    bool isGoalReached(
        geometry_msgs::PoseStamped robot_pose,
        double &dyaw);

    /**
     * @brief Get the orientation of the line segment between two points
     * @param p1 The first point
     * @param p2 The second point
     * @return The angle between the two points
     */
    geometry_msgs::Quaternion getOrientation(
        const geometry_msgs::Point & p1,
        const geometry_msgs::Point & p2);

    /**
     * @brief Calculate the turning radius of the robot given a target point
     * @param target_pose The target pose to calculate the turning radius
     * @return The turning radius of the robot
     */
    double calcTurningRadius(
        const geometry_msgs::PoseStamped & target_pose);

protected:
    /**
     * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
     * Points ineligible to be selected as a lookahead point if they are any of the following:
     * - Outside the local_costmap (collision avoidance cannot be assured)
     * @param pose pose to transform
     * @return Path in new frame
     */
    bool transformGlobalPlan(
        const geometry_msgs::PoseStamped & pose,
        nav_msgs::Path& transformed_plan);

    /**
     * @brief Get lookahead distance
     * @param cmd the current speed to use to compute lookahead point
     * @return lookahead distance
     */
    double getLookAheadDistance(const geometry_msgs::Twist &);

    /**
     * @brief Find the intersection a circle and a line segment.
     * This assumes the circle is centered at the origin.
     * If no intersection is found, a floating point error will occur.
     * @param p1 first endpoint of line segment
     * @param p2 second endpoint of line segment
     * @param r radius of circle
     * @return point of intersection
     */
    static geometry_msgs::Point circleSegmentIntersection(
        const geometry_msgs::Point & p1,
        const geometry_msgs::Point & p2,
        double r);

    /**
     * @brief checks for collision at projected pose
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @param theta orientation of Yaw
     * @return Whether in collision
     */
    bool inCollision(
        const double & x,
        const double & y,
        const double & theta);

    /**
     * @brief Whether collision is imminent
     * @param robot_pose Pose of robot
     * @param linear_vel linear velocity to forward project
     * @param angular_vel angular velocity to forward project
     * @param target_dist Distance to the lookahead point
     * @return Whether collision is imminent
     */
    bool isCollisionImminent(
        const geometry_msgs::PoseStamped & robot_pose,
        const double & linear_vel, const double & angular_vel,
        const double & target_dist);

    /**
        * @brief Get lookahead point
        * @param lookahead_dist Optimal lookahead distance
        * @param path Current global path
        * @return Lookahead point
        * @details The lookahead point could:
        * 1. Be discrete/interpolated
        * 2. Have a heading:
        *    a. Directly from path (discrete)
        *    b. Directly from path (interpolated) (if use_interpolation_ is true)
        *    c. Computed from path (if use_heading_from_path_ is false)
        *
        * Hence the flags use_interpolation_ and use_heading_from_path_
        * play important roles here.
        */
    geometry_msgs::PoseStamped getLookAheadPoint(const double &, const nav_msgs::Path &);

    /**
     * @brief Cost at a point
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @return Cost of pose in costmap
     */
    bool costAtPose(const double & x, const double & y, double &cost);

    double approachVelocityScalingFactor(
        const nav_msgs::Path & path
    ) const;

    /**
     * @brief Apply approach velocity scaling to the system
     * @param path Transformed global path
     * @param linear_vel robot command linear velocity input
     */
    void applyApproachVelocityScaling(
        const nav_msgs::Path & path,
        double & linear_vel
    ) const;

    /**
     * @brief Apply cost and curvature constraints to the system
     * @param curvature curvature of path
     * @param curr_speed Speed of robot
     * @param pose_cost cost at this pose
     * @param linear_vel robot command linear velocity input
     * @param path Transformed global path
     */
    void applyConstraints(
        const double & curvature, const geometry_msgs::Twist & curr_speed,
        const double & pose_cost, double & linear_vel, const nav_msgs::Path & path, double & sign);

    /**
     * @brief Whether robot should rotate to rough path heading
     * @param target_pose current lookahead point
     * @param angle_to_path Angle of robot output relative to lookahead point
     * @return Whether should rotate to path heading
     */
    bool shouldRotateToPath(
        const geometry_msgs::PoseStamped & target_pose,
        double & angle_to_path, double & sign);

    /**
     * @brief Whether robot should rotate to final goal orientation
     * @param target_pose current lookahead point
     * @return Whether should rotate to goal heading
     */
    bool shouldRotateToGoalHeading(double angle_to_goal);

    /**
     * @brief checks for the cusp position
     * @param pose Pose input to determine the cusp position
     * @return robot distance from the cusp
     */
    double getCuspDist(const nav_msgs::Path & transformed_plan);

    void rotateToGoal(double & linear_vel,
                      double & angular_vel,
                      const double & angle_to_goal);
    
    void rotateToHeading(double & linear_vel,
                         double & angular_vel,
                         const double & angle_to_path);

    /**
     * Get the greatest extent of the costmap in meters from the center.
     * @return max of distance from center in meters to edge of costmap
     */
    double getCostmapMaxExtent() const;

    void reconfigureCB(Config& config, uint32_t level);

    void runningCallback(const std_msgs::Bool::ConstPtr &msg);

    Parameters * params_;
    
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::OdometryHelperRos odom_helper_;
    geometry_msgs::PoseStamped goal_;
    geometry_msgs::PoseStamped prev_goal_;
    
    bool initialized_;
    bool is_running_;
    bool goal_reached_;
    bool check_xy_;
    double control_duration_;
    std::string controller_name_ = "VectorPursuitController";

    geometry_msgs::Twist last_cmd_vel_;

    nav_msgs::Path global_plan_;
    ros::Publisher global_path_pub_;
    ros::Publisher target_pub_;
    ros::Publisher target_arc_pub_;
    ros::Subscriber running_sub_;

    std::unique_ptr<FootprintCollisionChecker<costmap_2d::Costmap2D *>> collision_checker_;
    std::unique_ptr<vector_pursuit_controller::ParameterHandler> param_handler_;

    // Dynamic parameters handler
    std::mutex mutex_;
    ParamterConfigServer* dynamic_srv_;
};

} // namespace vector_pursuit_controller

#endif  // VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
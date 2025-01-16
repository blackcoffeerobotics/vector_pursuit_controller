// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "path_utils/path_utils.hpp"
#include "vector_pursuit_controller/vector_pursuit_controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class Controller : public vector_pursuit_controller::VectorPursuitController
{
public:
  Controller()
  : vector_pursuit_controller::VectorPursuitController() {}

  nav_msgs::msg::Path getPlan() {return global_plan_;}

  double getSpeed() {return desired_linear_vel_;}

  double getMinTurningRadius() {return min_turning_radius_;}
  void setMinTurningRadius(double r) {min_turning_radius_ = r;}

  void setVelocityScaledLookAhead() {use_velocity_scaled_lookahead_dist_ = true;}
  void setCostRegulationScaling() {use_cost_regulated_linear_velocity_scaling_ = true;}

  double getLookAheadDistanceWrapper(const geometry_msgs::msg::Twist & twist)
  {
    return getLookAheadDistance(twist);
  }

  double calcTurningRadiusWrappper(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    return calcTurningRadius(target_pose);
  }

  static geometry_msgs::msg::Point circleSegmentIntersectionWrapper(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r)
  {
    return circleSegmentIntersection(p1, p2, r);
  }

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
  }

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & target_pose, double & angle_to_path, double & sign)
  {
    return shouldRotateToPath(target_pose, angle_to_path, sign);
  }

  void rotateToHeadingWrapper(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
  {
    return rotateToHeading(linear_vel, angular_vel, angle_to_path, curr_speed);
  }

  void applyConstraintsWrapper(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost, double & linear_vel, nav_msgs::msg::Path & path)
  {
    double sign = 1.0;
    return applyConstraints(
      curvature, curr_speed, pose_cost,
      linear_vel, path, sign);
  }

  nav_msgs::msg::Path transformGlobalPlanWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformGlobalPlan(pose);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommandsWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker)
  {
    return computeVelocityCommands(pose, speed, goal_checker);
  }
};

TEST(VectorPursuitTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<Controller>();
  costmap->on_configure(rclcpp_lifecycle::State());
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->deactivate();
  ctrl->cleanup();

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(30, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.3);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}

using CircleSegmentIntersectionParam = std::tuple<
  std::pair<double, double>,
  std::pair<double, double>,
  double,
  std::pair<double, double>
>;

class CircleSegmentIntersectionTest
  : public ::testing::TestWithParam<CircleSegmentIntersectionParam>
{};

TEST_P(CircleSegmentIntersectionTest, circleSegmentIntersection)
{
  auto pair1 = std::get<0>(GetParam());
  auto pair2 = std::get<1>(GetParam());
  auto r = std::get<2>(GetParam());
  auto expected_pair = std::get<3>(GetParam());
  auto pair_to_point = [](std::pair<double, double> p) -> geometry_msgs::msg::Point {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = 0.0;
      return point;
    };
  auto p1 = pair_to_point(pair1);
  auto p2 = pair_to_point(pair2);
  auto actual = Controller::circleSegmentIntersectionWrapper(p1, p2, r);
  auto expected_point = pair_to_point(expected_pair);
  EXPECT_DOUBLE_EQ(actual.x, expected_point.x);
  EXPECT_DOUBLE_EQ(actual.y, expected_point.y);
  // Expect that the intersection point is actually r away from the origin
  EXPECT_DOUBLE_EQ(r, std::hypot(actual.x, actual.y));
}

INSTANTIATE_TEST_SUITE_P(
  InterpolationTest,
  CircleSegmentIntersectionTest,
  testing::Values(
    // Origin to the positive X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {2.0, 0.0},
  1.0,
  {1.0, 0.0}
},
    // Origin to the negative X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-2.0, 0.0},
  1.0,
  {-1.0, 0.0}
},
    // Origin to the positive Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  1.0,
  {0.0, 1.0}
},
    // Origin to the negative Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, -2.0},
  1.0,
  {0.0, -1.0}
},
    // non-origin to the X axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {4.0, 0.0},
  {-1.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // non-origin to the Y axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {0.0, 4.0},
  {0.0, -0.5},
  2.0,
  {0.0, 2.0}
},
    // origin to the positive X axis, on the circle
    CircleSegmentIntersectionParam{
  {2.0, 0.0},
  {0.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // origin to the positive Y axis, on the circle
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  2.0,
  {0.0, 2.0}
},
    // origin to the upper-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, 8.0},
  5.0,
  {3.0, 4.0}
},
    // origin to the lower-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, -8.0},
  5.0,
  {-3.0, -4.0}
},
    // origin to the upper-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, 8.0},
  5.0,
  {-3.0, 4.0}
},
    // origin to the lower-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, -8.0},
  5.0,
  {3.0, -4.0}
}
));

TEST(VectorPursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::Twist twist;

  // test getLookAheadDistance
  double rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);  // default lookahead_dist

  // shouldn't be a function of speed
  twist.linear.x = 10.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);

  // now it should be a function of velocity, max out
  ctrl->setVelocityScaledLookAhead();
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.9);  // 10 speed maxes out at max_lookahead_dist

  // check normal range
  twist.linear.x = 0.35;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_NEAR(rtn, 0.525, 0.0001);  // 1.5 * 0.35

  // check minimum range
  twist.linear.x = 0.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.3);

  // test getLookAheadPoint
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test getting next closest point without interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test end of path
  dist = 100.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 9.0);

  // Test without use heading from path
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);

  dist = 4.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(true)));
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(true)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);

  // Test without use heading from path with interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);

  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);

  // Path not starting at the robot pose
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i + 1);
  }

  // if the first pose is ahead of the lookahead distance, take the first pose discretely
  dist = 0.7;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // If no pose is far enough, take the last pose discretely
  dist = 11.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 10.0);
}

TEST(VectorPursuitTest, calcTurnRadius) {
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::PoseStamped carrot;

  // directly ahead
  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.0;

  EXPECT_EQ(ctrl->calcTurningRadiusWrappper(carrot), std::numeric_limits<double>::max());

  // directly behind
  carrot.pose.position.x = -0.5;
  carrot.pose.position.y = 0.0;

  EXPECT_EQ(ctrl->calcTurningRadiusWrappper(carrot), ctrl->getMinTurningRadius());

  // diagonal
  carrot.pose.position.x = 0.1;
  carrot.pose.position.y = 0.1;

  ctrl->setMinTurningRadius(0.0);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.11, 0.01);

  // beside
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.1;

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.05, 0.01);

  // beside, rotated 90 degrees
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.1;

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -M_PI);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.04, 0.01);
}

TEST(VectorPursuitTest, rotateTests)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn;
  double sign = 1.0;

  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), true);

  // rotateToHeading
  double lin_v = 10.0;
  double ang_v = 0.5;
  double angle_to_path = 0.4;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.angular.z = 1.75;

  // basic full speed at a speed
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(lin_v, 0.0);
  EXPECT_EQ(ang_v, 1.8);

  // negative direction
  angle_to_path = -0.4;
  curr_speed.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, -1.8);

  // kinematic clamping, no speed, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.16, 0.01);

  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 1.16, 0.01);

  angle_to_path = -0.4;
  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.84, 0.01);
}

TEST(VectorPursuitTest, applyConstraints)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  constexpr double desired_linear_vel = 1.0;
  nav2_util::declare_parameter_if_not_declared(
    node,
    name + ".desired_linear_vel",
    rclcpp::ParameterValue(desired_linear_vel));

  ctrl->configure(node, name, tf, costmap);

  auto no_approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.6 + 1.0)
  });

  double curvature = 0.5;
  geometry_msgs::msg::Twist curr_speed;
  double pose_cost = 0.0;
  double linear_vel = desired_linear_vel;

  // test curvature regulation
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.35);  // max linear acceleration contraint

  linear_vel = 1.0;
  curvature = 3.6;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_LT(linear_vel, 0.5);  // lower by curvature

  linear_vel = 1.0;
  curvature = 1000.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.05);  // min out by curvature

  // Approach velocity scaling on a path with no distance left
  auto approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.0)
  });

  linear_vel = 1.0;
  curvature = 0.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, linear_vel, approach_path);
  EXPECT_NEAR(linear_vel, 0.05, 0.01);  // min out on min approach velocity

  // now try with cost regulation
  ctrl->setCostRegulationScaling();
  curvature = 0.0;

  // min changable cost
  pose_cost = 1;
  linear_vel = 0.5;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.498, 0.01);

  // max changing cost
  pose_cost = 127;
  curr_speed.linear.x = 0.255;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.280, 0.01);

  // over max cost thresh
  pose_cost = 200;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.086, 0.01);

  // test kinematic clamping
  pose_cost = 200;
  curr_speed.linear.x = 1.0;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.05);
}

TEST(VectorPursuitTest, testDynamicParameter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl =
    std::make_unique<vector_pursuit_controller::VectorPursuitController>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  ctrl->configure(node, "test", tf, costmap);
  ctrl->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.k", 1.5),
      rclcpp::Parameter("test.desired_linear_vel", 1.0),
      rclcpp::Parameter("test.transform_tolerance", 0.5),
      rclcpp::Parameter("test.lookahead_dist", 1.0),
      rclcpp::Parameter("test.min_lookahead_dist", 6.0),
      rclcpp::Parameter("test.max_lookahead_dist", 7.0),
      rclcpp::Parameter("test.lookahead_time", 1.8),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 18.0),
      rclcpp::Parameter("test.rotate_to_heading_min_angle", 0.7),
      rclcpp::Parameter("test.min_linear_velocity", 1.0),
      rclcpp::Parameter("test.min_turning_radius", 0.0),
      rclcpp::Parameter("test.max_angular_accel", 3.0),
      rclcpp::Parameter("test.max_lateral_accel", 2.0),
      rclcpp::Parameter("test.max_linear_accel", 2.5),
      rclcpp::Parameter("test.max_allowed_time_to_collision_up_to_target", 5.0),
      rclcpp::Parameter("test.approach_velocity_scaling_dist", 10.0),
      rclcpp::Parameter("test.min_approach_linear_velocity", 0.6),
      rclcpp::Parameter("test.cost_scaling_dist", 2.0),
      rclcpp::Parameter("test.cost_scaling_gain", 4.0),
      rclcpp::Parameter("test.inflation_cost_scaling_factor", -1.0),
      rclcpp::Parameter("test.inflation_cost_scaling_factor", 1.0),
      rclcpp::Parameter("test.use_collision_detection", true),
      rclcpp::Parameter("test.use_velocity_scaled_lookahead_dist", false),
      rclcpp::Parameter("test.use_cost_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.use_rotate_to_heading", false),
      rclcpp::Parameter("test.use_interpolation", true),
      rclcpp::Parameter("test.use_heading_from_path", false),
      rclcpp::Parameter("test.allow_reversing", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.k").as_double(), 1.5);
  EXPECT_EQ(node->get_parameter("test.desired_linear_vel").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("test.lookahead_dist").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.min_lookahead_dist").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_time").as_double(), 1.8);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_min_angle").as_double(), 0.7);
  EXPECT_EQ(node->get_parameter("test.min_linear_velocity").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.min_turning_radius").as_double(), 0.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("test.max_lateral_accel").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.max_linear_accel").as_double(), 2.5);
  EXPECT_EQ(
    node->get_parameter(
      "test.max_allowed_time_to_collision_up_to_target").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("test.approach_velocity_scaling_dist").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.min_approach_linear_velocity").as_double(), 0.6);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_gain").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.inflation_cost_scaling_factor").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.use_collision_detection").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_velocity_scaled_lookahead_dist").as_bool(), false);
  EXPECT_EQ(
    node->get_parameter(
      "test.use_cost_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_rotate_to_heading").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_interpolation").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_heading_from_path").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.allow_reversing").as_bool(), true);
}

class TransformGlobalPlanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctrl_ = std::make_shared<Controller>();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  }

  void configure_costmap(uint16_t width, double resolution)
  {
    constexpr char costmap_frame[] = "test_costmap_frame";
    constexpr char robot_frame[] = "test_robot_frame";

    auto results = costmap_->set_parameters(
    {
      rclcpp::Parameter("global_frame", costmap_frame),
      rclcpp::Parameter("robot_base_frame", robot_frame),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", width),
      rclcpp::Parameter("resolution", resolution)
    });
    for (const auto & result : results) {
      EXPECT_TRUE(result.successful) << result.reason;
    }

    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
  }

  void configure_controller(double max_robot_pose_search_dist)
  {
    std::string plugin_name = "test_vp";
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".max_robot_pose_search_dist",
      rclcpp::ParameterValue(max_robot_pose_search_dist));
    ctrl_->configure(node_, plugin_name, tf_buffer_, costmap_);
  }

  void setup_transforms(geometry_msgs::msg::Point & robot_position)
  {
    transform_time_ = node_->get_clock()->now();
    // Note: transforms go parent to child

    // We will have a separate path and costmap frame for completeness,
    // but we will leave them cooincident for convenience.
    geometry_msgs::msg::TransformStamped path_to_costmap;
    path_to_costmap.header.frame_id = PATH_FRAME;
    path_to_costmap.header.stamp = transform_time_;
    path_to_costmap.child_frame_id = COSTMAP_FRAME;
    path_to_costmap.transform.translation.x = 0.0;
    path_to_costmap.transform.translation.y = 0.0;
    path_to_costmap.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped costmap_to_robot;
    costmap_to_robot.header.frame_id = COSTMAP_FRAME;
    costmap_to_robot.header.stamp = transform_time_;
    costmap_to_robot.child_frame_id = ROBOT_FRAME;
    costmap_to_robot.transform.translation.x = robot_position.x;
    costmap_to_robot.transform.translation.y = robot_position.y;
    costmap_to_robot.transform.translation.z = robot_position.z;

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms = {
      path_to_costmap,
      costmap_to_robot
    };
    for (const auto & transform : tf_message.transforms) {
      tf_buffer_->setTransform(transform, "test", false);
    }
    tf_buffer_->setUsingDedicatedThread(true);  // lying to let it do transforms
  }

  static constexpr char PATH_FRAME[] = "test_path_frame";
  static constexpr char COSTMAP_FRAME[] = "test_costmap_frame";
  static constexpr char ROBOT_FRAME[] = "test_robot_frame";

  std::shared_ptr<Controller> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Time transform_time_;
};

// This tests that not only should nothing get pruned on a costmap
// that contains the entire global_plan, and also that it doesn't skip to the end of the path
// which is closer to the robot pose than the start.
TEST_F(TransformGlobalPlanTest, no_pruning_on_large_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // A really big costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  configure_controller(5.0);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double spacing = 0.1;
  constexpr double circle_radius = 1.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan

  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), global_plan.poses.size());
}

// This plan shouldn't get pruned because of the costmap,
// but should be half pruned because it is halfway around the circle
TEST_F(TransformGlobalPlanTest, transform_start_selection)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 4.0;  // on the other side of the circle
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A really big costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), global_plan.poses.size() / 2, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// This should throw an exception when all poses are outside of the costmap
TEST_F(TransformGlobalPlanTest, all_poses_outside_of_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = 1000.0;
  robot_pose.pose.position.y = 1000.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(10u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  EXPECT_THROW(ctrl_->transformGlobalPlanWrapper(robot_pose), nav2_core::PlannerException);
}

// Should shortcut the circle if the circle is shorter than max_robot_pose_search_dist
TEST_F(TransformGlobalPlanTest, good_circle_shortcut)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least the circumference: 2*pi*r ~= 12
  constexpr double max_robot_pose_search_dist = 15.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 1, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Simple costmap pruning on a straight line
TEST_F(TransformGlobalPlanTest, costmap_pruning)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Should prune out later portions of the path that come back into the costmap
TEST_F(TransformGlobalPlanTest, prune_after_leaving_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length),
    std::make_unique<path_utils::LeftTurnAround>(1.0),
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  // This should be essentially the same as the regular straight path
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

class ComputeVelocityCommandsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testVP");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    ctrl_ = std::make_shared<Controller>();
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    checker_.initialize(node_, "fake_checker", costmap_);
  }

  void configure_costmap(uint16_t width, double resolution)
  {
    auto results = costmap_->set_parameters(
    {
      rclcpp::Parameter("global_frame", PATH_FRAME),
      rclcpp::Parameter("robot_base_frame", ROBOT_FRAME),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", width),
      rclcpp::Parameter("resolution", resolution)
    });
    for (const auto & result : results) {
      EXPECT_TRUE(result.successful) << result.reason;
    }

    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
  }

  void configure_controller(double max_robot_pose_search_dist, bool allow_reversing)
  {
    std::string plugin_name = "test_vp";
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".max_robot_pose_search_dist",
      rclcpp::ParameterValue(max_robot_pose_search_dist));
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".allow_reversing",
      rclcpp::ParameterValue(allow_reversing));
    ctrl_->configure(node_, plugin_name, tf_buffer_, costmap_);
  }

  void setup_transforms(geometry_msgs::msg::Point & robot_position)
  {
    transform_time_ = node_->get_clock()->now();
    // Note: transforms go parent to child

    // We will have a separate path and costmap frame for completeness,
    // but we will leave them cooincident for convenience.
    geometry_msgs::msg::TransformStamped path_to_costmap;
    path_to_costmap.header.frame_id = PATH_FRAME;
    path_to_costmap.header.stamp = transform_time_;
    path_to_costmap.child_frame_id = COSTMAP_FRAME;
    path_to_costmap.transform.translation.x = 0.0;
    path_to_costmap.transform.translation.y = 0.0;
    path_to_costmap.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped costmap_to_robot;
    costmap_to_robot.header.frame_id = COSTMAP_FRAME;
    costmap_to_robot.header.stamp = transform_time_;
    costmap_to_robot.child_frame_id = ROBOT_FRAME;
    costmap_to_robot.transform.translation.x = robot_position.x;
    costmap_to_robot.transform.translation.y = robot_position.y;
    costmap_to_robot.transform.translation.z = robot_position.z;

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms = {
      path_to_costmap,
      costmap_to_robot
    };
    for (const auto & transform : tf_message.transforms) {
      tf_buffer_->setTransform(transform, "test", false);
    }
    tf_buffer_->setUsingDedicatedThread(true);  // lying to let it do transforms
  }

  static constexpr char PATH_FRAME[] = "test_path_frame";
  static constexpr char COSTMAP_FRAME[] = "test_costmap_frame";
  static constexpr char ROBOT_FRAME[] = "test_robot_frame";

  std::shared_ptr<Controller> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav2_controller::SimpleGoalChecker checker_;

  rclcpp::Time transform_time_;
};

TEST_F(ComputeVelocityCommandsTest, straightLineForward)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.position.y = 1.0;
  robot_pose.pose.position.z = 0.0;

  // setup
  setup_transforms(robot_pose.pose.position);
  configure_costmap(50u, 0.1);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist, false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path path;
  path.header.frame_id = PATH_FRAME;
  path.header.stamp = transform_time_;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].header.frame_id = PATH_FRAME;
    path.poses[i].header.stamp = transform_time_;
    path.poses[i].pose.position.y = 1.0;
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  ctrl_->setPlan(path);
  ctrl_->activate();

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.angular.z = 0.0;

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(robot_pose, robot_velocity, &checker_);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.1);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.01);
}

TEST_F(ComputeVelocityCommandsTest, straightLineBackward)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 25.0;
  robot_pose.pose.position.y = 25.0;
  robot_pose.pose.position.z = 0.0;

  // setup
  setup_transforms(robot_pose.pose.position);
  configure_costmap(50u, 0.1);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist, true);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path path;
  path.header.frame_id = PATH_FRAME;
  path.header.stamp = transform_time_;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].header.frame_id = PATH_FRAME;
    path.poses[i].header.stamp = transform_time_;
    path.poses[i].pose.position.y = 25.0;
    path.poses[i].pose.position.x = static_cast<double>(25 - i);
  }

  ctrl_->setPlan(path);
  ctrl_->activate();

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.angular.z = 0.0;

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(robot_pose, robot_velocity, &checker_);
  EXPECT_EQ(cmd_vel.twist.linear.x, -0.1);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.01);
}

TEST_F(ComputeVelocityCommandsTest, rotateToHeading)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 25.0;
  robot_pose.pose.position.y = 25.0;
  robot_pose.pose.position.z = 0.0;

  // setup
  setup_transforms(robot_pose.pose.position);
  configure_costmap(50u, 0.1);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist, false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path path;
  path.header.frame_id = PATH_FRAME;
  path.header.stamp = transform_time_;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].header.frame_id = PATH_FRAME;
    path.poses[i].header.stamp = transform_time_;
    path.poses[i].pose.position.y = static_cast<double>(30 + i);
    path.poses[i].pose.position.x = static_cast<double>(25 + i);
  }

  ctrl_->setPlan(path);
  ctrl_->activate();

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.angular.z = 0.0;

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(robot_pose, robot_velocity, &checker_);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.16, 0.01);
}

#ifndef VECTOR_PURSUIT_CONTROLLER__UTILS_HPP__
#define VECTOR_PURSUIT_CONTROLLER__UTILS_HPP__

#include <geometry_msgs/Point.h>
#include <tf2/utils.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>

namespace vector_pursuit_controller
{

/**
 * @brief Clamps a value within a specified range.
 * @param value          The value to be clamped.
 * @param low            The lower bound of the range.
 * @param high           The upper bound of the range.
 * @return const T&      The clamped value within the specified range.
 */
template <typename T>
const T& clamp(const T& value, const T& low, const T& high)
{
  return std::max(low, std::min(value, high));
}

inline double hypot(double x, double y, double z)
{
    return (std::sqrt(x*x + y*y + z*y));
}

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::Point & pos1,
  const geometry_msgs::Point & pos2,
  const bool is_3d = false)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}


/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::Pose & pos1,
  const geometry_msgs::Pose & pos2,
  const bool is_3d = false)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::PoseStamped & pos1,
  const geometry_msgs::PoseStamped & pos2,
  const bool is_3d = false)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::Pose2D & pos1,
  const geometry_msgs::Pose2D & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp <= lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

/**
 * @brief Transform a PoseStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @return True if successful transform
 */
inline bool transformPose(
    tf2_ros::Buffer* tf,
    const std::string frame,
    const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose,
    double transform_tolerance
)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame, ros::Duration(transform_tolerance));
  }
  catch(tf2::TransformException& ex)
  {
    ROS_ERROR(
      "VectorPursuitController: Failed when transform to '%s' from '%s' with error: %s"
      , in_pose.header.frame_id.c_str(), frame.c_str(), ex.what()
    );
    return false;
  }
  out_pose.header.frame_id = frame;
  return true;
}

}  // namespace geometry_utils

#endif // VECTOR_PURSUIT_CONTROLLER__UTILS_HPP__
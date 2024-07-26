# Vector Pursuit Controller

## About

This [ROS2 Humble](https://docs.ros.org/en/humble/index.html) package contains a plugin for the [Nav2 Controller Server](https://docs.nav2.org/configuration/packages/configuring-controller-server.html) that implements the [Vector Pursuit](https://apps.dtic.mil/sti/pdfs/ADA468928.pdf) path tracking algorithm. It leverages [Screw Theory](https://en.wikipedia.org/wiki/Screw_theory) to achieve smooth path tracking and comes with active collision detection. Like the [Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)(RPP) Controller, it tracks a look-ahead point on the path to be followed but offers geometrically-meaningful orientation tracking as well. This fixes many of the shortcomings of RPP like lower maximum speed and poor turning performance while keeping computation demands low.

[video]()

## Get Started
This is a simple 5-step example that uses [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot) in a Gazebo Fortress simulation with a default Nav2 Stack. An experienced reader can skip to the [Usage and Configuration](#usage-and-configuration) section for minimal, to-the-point instructions.

1. Create a ROS2 workspace and clone the repositories into this workspace.
    ```bash
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    
    git clone https://github.com/blackcoffeerobotics/vector_pursuit_controller.git
    git clone https://github.com/blackcoffeerobotics/bcr_bot.git

    # install package dependencies
    cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y
    ```

2. Install other ROS2 dependencies required for this tutorial.
    ```bash

    # fortress
    sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces 

    # nav2
    sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
    ```

3. Edit the [nav2_params.yaml](https://github.com/blackcoffeerobotics/bcr_bot/blob/ros2/config/nav2_params.yaml) file in the bcr_bot repository to replace `controller_server` parameters, copy these parameters from the [default parameters](#default-parameters) section.

4. Build the workspace and source it.
    ```bash
    cd ~/ros2_ws && colcon build && source install/setup.bash
    ``` 

5. Run the nodes in two different terminals.
    ```bash
    # in terminal 1
    ros2 launch bcr_bot ign.launch.py

    # in terminal 2, make sure to source the workspace
    ros2 launch bcr_bot nav2.launch.py
    ```

## Usage and Configuration
These are precise instructions to use the Vector Pursuit controller plugin. The controller server parameters need to be edited to include the vector pursuit plugin along with its [configuration](#default-parameters). Nav2's controller server supports multiple controller plugins at the same time and instructions for setting it up can be found in the [official docs](https://docs.nav2.org/configuration/packages/configuring-controller-server.html).

### Default Parameters
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.25
      movement_time_allowance: 10.0

    # Goal checker parameters
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    
    FollowPath:
      plugin: "vector_pursuit_controller::VectorPursuitController"
      p_gain: 8.0
      desired_linear_vel: 0.5
      min_turning_radius: 0.2
      lookahead_dist: 0.4
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_linear_velocity: 0.05
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      max_allowed_time_to_collision_up_to_target: 1.0
      use_collision_detection: true
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.5
      max_angular_accel: 3.2
      max_linear_accel: 2.0
      max_lateral_accel: 0.2
      max_robot_pose_search_dist: 10.0
      use_interpolation: true
      use_heading_from_path: false
```

### Configuration

The following parameters can be adjusted to optimize the robot's performance:

| Parameter                          | Description                                                     |
|------------------------------------|-----------------------------------------------------------------|
| `p_gain`                           | Proportional constant for controller                            |
| `desired_linear_vel`               | Maximum desired linear velocity                                 |
| `min_lookahead_dist`               | Minimum lookahead distance                                      |
| `max_lookahead_dist`               | Maximum lookahead distance                                      |
| `lookahead_time`                   | Time factor for velocity-scaled lookahead distance              |
| `rotate_to_heading_angular_vel`    | Angular velocity for rotating to heading                        |
| `rotate_to_heading_min_angle`      | Minimum angle to trigger rotate-to-heading behavior             |
| `min_linear_velocity`              | Minimum velocity achievable                                     |
| `min_turning_radius`               | Minimum turning radius                                          |
| `max_angular_accel`                | Maximum angular acceleration                                    |
| `max_lateral_accel`                | Maximum lateral acceleration                                    |
| `max_linear_accel`                 | Maximum linear acceleration                                     |
| `max_allowed_time_to_collision_up_to_target` | Maximum time allowed for collision checking           |
| `cost_scaling_dist`                | Distance for cost-based velocity scaling                        |
| `cost_scaling_gain`                | Gain factor for cost-based velocity scaling                     |
| `inflation_cost_scaling_factor`    | Factor for inflation cost scaling                               |
| `use_collision_detection`          | Enable/disable collision detection                              |
| `use_velocity_scaled_lookahead_dist` | Enable/disable velocity-scaled lookahead distance             |
| `use_cost_regulated_linear_velocity_scaling` | Enable/disable cost-regulated linear velocity scaling |
| `use_rotate_to_heading`            | Enable/disable rotate-to-heading behavior. Will override reversing if both are enabled                    |
| `use_interpolation`                | Calculate lookahead point exactly at the lookahead distance.    |
| `use_heading_from_path`            | Will use the orientation of the path poses, if set to true. Will compute orientations from path (x,y) points otherwise, only set to true if using a planner that takes robot heading into  account like [Smach Planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)|
| `allow_reversing`                | Will move in reverse if the lookahead point is behind the robot. |

### Note on Tuning:
The min turning radius of the controller should be equal to or less than the min turning radius of the planner (in case it is available). This ensures the controller can follow the path generated by the planner and not get stuck in a loop.

## Vector Pursuit Algorithm
Vector pursuit is a geometric path-tracking method that uses the theory of screws. It is similar to other geometric methods in that a look-ahead distance is used to define a current goal point, and then geometry is used to determine the desired motion of the vehicle. On the other hand, it is different from current geometric path-tracking methods, such as follow-the-carrot or pure pursuit, which do not use the orientation at the look-ahead point. Proportional path tracking is a geometric method that does use the orientation at the look-ahead point. This method adds the current position error multiplied by some gain to the current orientation error multiplied by some gain, and therefore becomes geometrically meaningless since terms with different units are added. Vector pursuit uses both the location and orientation of
the look-ahead point while remaining geometrically meaningful. 
It first calculates two instantaneous screws for translation and rotation, and then combines them to form a single screw representing the required motion. Then it calculates a desired turning radius from the resultant screw.
/*********************************************************************
MIT License

Copyright (c) 2022 neobotix gmbh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *********************************************************************/

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <optional>

#include "tf2_ros/static_transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "neo_perception2/contour_matching.hpp"

#include "neo_srvs2/srv/set_safety_field.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class NeoDocking
  : public rclcpp::Node
{
public:
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions send_goal_options;

  using NavigateToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions nav_to_goal_options;

  std::shared_ptr<rclcpp::Node> safety_client_node_;

  NeoDocking()
  : Node("neo_docking2")
  {
    this->declare_parameter<double>("offset_x", 0.70);
    this->declare_parameter<double>("offset_y", -0.37);
    this->declare_parameter<double>("offset_yaw", 0.03);
    this->declare_parameter<double>("undock_dist", 0.5);
    this->declare_parameter<double>("pre_dock_dist", 0.5);
    this->declare_parameter<double>("approach_distance", 0.37);
    this->declare_parameter<double>("distance_tolerance", 0.005);
    this->declare_parameter<std::string>("scan_topic", "/scan");

    this->get_parameter("offset_x", offset_x_);
    this->get_parameter("offset_y", offset_y_);
    this->get_parameter("offset_yaw", offset_yaw_);
    this->get_parameter("scan_topic", scan_topic_);
    this->get_parameter("undock_dist", undock_dist_);
    this->get_parameter("pre_dock_dist", pre_dock_dist_);
    this->get_parameter("approach_distance", approach_distance_);
    this->get_parameter("distance_tolerance", distance_tolerance_);

    bool parameters_valid = true;
    const auto require_finite = [this, &parameters_valid](
      const char * name, const double value)
      {
        if (!std::isfinite(value)) {
          RCLCPP_FATAL(this->get_logger(), "Parameter '%s' must be finite", name);
          parameters_valid = false;
        }
      };
    require_finite("offset_x", offset_x_);
    require_finite("offset_y", offset_y_);
    require_finite("offset_yaw", offset_yaw_);
    require_finite("undock_dist", undock_dist_);
    require_finite("pre_dock_dist", pre_dock_dist_);
    require_finite("approach_distance", approach_distance_);
    require_finite("distance_tolerance", distance_tolerance_);

    if (undock_dist_ <= 0.0) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'undock_dist' must be greater than zero");
      parameters_valid = false;
    }
    if (pre_dock_dist_ <= 0.0) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'pre_dock_dist' must be greater than zero");
      parameters_valid = false;
    }
    if (approach_distance_ <= 0.0) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'approach_distance' must be greater than zero");
      parameters_valid = false;
    }
    if (distance_tolerance_ < 0.0) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'distance_tolerance' must be zero or greater");
      parameters_valid = false;
    }
    if (scan_topic_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'scan_topic' must not be empty");
      parameters_valid = false;
    }
    if (!parameters_valid) {
      rclcpp::shutdown();
      return;
    }

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // call to dock
    docking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "go_and_dock", std::bind(&NeoDocking::dock, this, _1, _2));
    // call to undock
    undocking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "undock_and_arm", std::bind(&NeoDocking::undock, this, _1, _2));

    // Client for handling nbx_safety.
    safety_client_node_ = std::make_shared<rclcpp::Node>("safety_client_node");
    set_safety_field_client_ = safety_client_node_->create_client
      <neo_srvs2::srv::SetSafetyField>("set_safety_field");

    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    client_node_ = std::make_shared<rclcpp::Node>("docking_client_node");

    contour_matching = std::make_shared<ContourMatching>(
      this->create_sub_node("perception"),
      scan_topic_,
      offset_x_,
      offset_y_,
      offset_yaw_);
    // Publish static transforms once at startup
    this->make_transforms();
    dock_poses_.reserve(2);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    waypoint_follower_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      client_node_,
      "follow_waypoints");

    navigate_to_pose_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        client_node_,
        "navigate_to_pose");

    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    // Seperate thread for recieving the result_callback
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NeoDocking::helper_thread, this));

    send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

    nav_to_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  }

  bool helper_set_safety_field(const uint32_t field_id)
  {
    auto request = std::make_shared<neo_srvs2::srv::SetSafetyField::Request>();
    request->field_id = field_id;

    // Check service is available
    while (!set_safety_field_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(safety_client_node_->get_logger(),
        "set_safety_field service not found. Exiting.");
        return false;
      }
      RCLCPP_INFO(safety_client_node_->get_logger(),
      "waiting for set_safety_field service to be available");
    }

    auto result = set_safety_field_client_->async_send_request(request);

    if (result.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      // The request is complete, process the result
      auto response = result.get();
      if (response->success) {
        RCLCPP_INFO(
          safety_client_node_->get_logger(), "Safety field set to %u", field_id);
        return true;
      } else {
        RCLCPP_WARN(
          safety_client_node_->get_logger(),
          "Request to set safety field to %u failed", field_id);
        return false;
      }
    } else {
      // The request did not complete within the timeout
      RCLCPP_ERROR(
        safety_client_node_->get_logger(),
        "Timeout while setting safety field to %u", field_id
      );
      return false;
    }
  }

  void start_final_approach()
  {
    RCLCPP_INFO(this->get_logger(), "Starting odom-based final approach");
    geometry_msgs::msg::TransformStamped robot_pose;
    rclcpp::Rate loop_rate(100);
    rclcpp::Time set_approach_time;
    rclcpp::Time last_velocity_update = this->get_clock()->now();
    double commanded_linear_velocity = 0.0;

    constexpr double max_acceleration = 0.10;  // m/s^2
    constexpr double max_deceleration = 0.15;  // m/s^2
    auto smooth_velocity = [&](const double target_velocity) {
        const auto now = this->get_clock()->now();
        const double dt = std::clamp(
          (now - last_velocity_update).seconds(), 0.0, 0.1);
        last_velocity_update = now;

        const double rate_limit = target_velocity > commanded_linear_velocity ?
          max_acceleration : max_deceleration;
        const double max_velocity_change = rate_limit * dt;
        commanded_linear_velocity += std::clamp(
          target_velocity - commanded_linear_velocity,
          -max_velocity_change, max_velocity_change);
        return commanded_linear_velocity;
      };

    if (!frozen_docking_pose_in_odom_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot start final approach: no docking pose was frozen after contour matching");
      geometry_msgs::msg::Twist stop_twist;
      vel_pub->publish(stop_twist);
      on_process_ = false;
      nav_task_finished_ = false;
      return;
    }

    const auto & docking_pose_in_odom = *frozen_docking_pose_in_odom_;

    const auto & docking_rotation = docking_pose_in_odom.transform.rotation;
    const tf2::Quaternion docking_orientation(
      docking_rotation.x, docking_rotation.y, docking_rotation.z, docking_rotation.w);
    const auto & docking_translation = docking_pose_in_odom.transform.translation;
    const tf2::Transform frozen_docking_transform(
      docking_orientation,
      tf2::Vector3(docking_translation.x, docking_translation.y, docking_translation.z));
    const tf2::Transform docking_from_odom = frozen_docking_transform.inverse();

    while (rclcpp::ok() && !goal_reached_) {
      try {
        robot_pose = buffer_->lookupTransform(
          final_approach_frame_, base_link_, tf2::TimePointZero);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          this->get_logger(), "No transform found between %s and %s: %s",
          final_approach_frame_.c_str(), base_link_.c_str(), ex.what());
        geometry_msgs::msg::Twist stop_twist;
        commanded_linear_velocity = 0.0;
        vel_pub->publish(stop_twist);
        on_process_ = false;
        nav_task_finished_ = false;
        break;
      }

      // Express the live odom position in the frozen docking frame and use only
      // its signed x-distance. No map transform is involved in this final loop.
      const auto & robot_translation = robot_pose.transform.translation;
      const tf2::Vector3 robot_position_in_docking = docking_from_odom *
        tf2::Vector3(robot_translation.x, robot_translation.y, robot_translation.z);
      const double remaining_distance = -robot_position_in_docking.x();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Final approach x-distance in docking frame: %.4f m", remaining_distance);
      geometry_msgs::msg::Twist twist_vel;

      // additionaly layer check if docking has completed
      if (remaining_distance <= 0.015) {
        RCLCPP_INFO(client_node_->get_logger(), "Check 1: Docking finished");
        twist_vel.linear.x = 0.0;   // Setting 0 velocity
        commanded_linear_velocity = 0.0;
        vel_pub->publish(twist_vel);
        on_process_ = false;
        nav_task_finished_ = false;
        goal_reached_ = true;
      }

      if (!set_approaching_ && !goal_reached_) {
        if (remaining_distance > approach_distance_ + distance_tolerance_) {
          RCLCPP_INFO_ONCE(client_node_->get_logger(), "Navigating in approach buffer");
          twist_vel.linear.x = smooth_velocity(0.05);
          vel_pub->publish(twist_vel);
        } else {
          twist_vel.linear.x = smooth_velocity(0.0);
          vel_pub->publish(twist_vel);

          if (std::abs(commanded_linear_velocity) < 1e-3) {
            commanded_linear_velocity = 0.0;
            twist_vel.linear.x = 0.0;
            vel_pub->publish(twist_vel);
            // Safety field 8 was selected before docking navigation started.
            set_approaching_ = true;
            set_approach_time = this->get_clock()->now();
            last_velocity_update = set_approach_time;
          }
        }
      }

      /** Set the conditions for the robot to dock. The laser-derived docking
        * pose is fixed in odom so map localization corrections cannot move the
        * final stopping target. **/

      if (set_approaching_ && !goal_reached_) {
        auto lapsed_time = (this->get_clock()->now() - set_approach_time).seconds();
        if (lapsed_time > 3.0) {
          if (remaining_distance > 0.01 && lapsed_time < 12.0) {
            constexpr double docking_speed = 0.05;
            constexpr double slowdown_distance = 0.045;
            constexpr double stop_distance = 0.01;
            const double normalized_distance = std::clamp(
              (remaining_distance - stop_distance) /
              (slowdown_distance - stop_distance),
              0.0, 1.0);
            const double smoothstep = normalized_distance * normalized_distance *
              (3.0 - 2.0 * normalized_distance);
            twist_vel.linear.x = smooth_velocity(docking_speed * smoothstep);
            vel_pub->publish(twist_vel);
          } else {
            RCLCPP_INFO(client_node_->get_logger(), "Check 2: Docking finished");
            twist_vel.linear.x = 0.0;   // Setting 0 velocity
            commanded_linear_velocity = 0.0;
            vel_pub->publish(twist_vel);
            on_process_ = false;
            nav_task_finished_ = false;
            goal_reached_ = true;
            RCLCPP_INFO(client_node_->get_logger(),
              "Check 2: Docking finished - remaining x-distance: %f",
              remaining_distance);
          }
        }
      }
      loop_rate.sleep();
    }
    set_approaching_ = false;
    goal_reached_ = false;
  }

  void helper_thread()
  {
    // Only have the result callback enabled if the control is not in the stage
    if (!nav_task_finished_) {
      send_goal_options.result_callback =
        std::bind(&NeoDocking::result_callback, this, _1);
      nav_to_goal_options.result_callback =
        std::bind(&NeoDocking::result_pre_dock_callback, this, _1);
    }
    rclcpp::spin_some(client_node_);
  }

private:
  bool freeze_final_docking_pose()
  {
    try {
      frozen_docking_pose_in_odom_ = buffer_->lookupTransform(
        final_approach_frame_, docking_station_, tf2::TimePointZero);
    } catch (const std::exception & ex) {
      frozen_docking_pose_in_odom_.reset();
      RCLCPP_ERROR(
        this->get_logger(), "Cannot freeze docking pose from %s to %s: %s",
        final_approach_frame_.c_str(), docking_station_.c_str(), ex.what());
      return false;
    }

    const auto & frozen_pose = *frozen_docking_pose_in_odom_;
    RCLCPP_INFO(
      this->get_logger(), "Frozen final docking target in %s at x=%.3f, y=%.3f",
      final_approach_frame_.c_str(),
      frozen_pose.transform.translation.x,
      frozen_pose.transform.translation.y);
    return true;
  }

  inline double euclidean_distance(
    geometry_msgs::msg::TransformStamped & pos1,
    const geometry_msgs::msg::TransformStamped & pos2)
  {
    double dx = pos1.transform.translation.x - pos2.transform.translation.x;
    double dy = pos1.transform.translation.y - pos2.transform.translation.y;

    return std::hypot(dx, dy);
  }

  void result_pre_dock_callback(const NavigateToPoseGoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "pre dock succeded");
      geometry_msgs::msg::Pose init_guess;
      contour_matching->startMatching();

      RCLCPP_INFO(
        this->get_logger(),
        "resetting initial guess");

      init_guess.position.x = 0.0;
      init_guess.position.y = -offset_y_;
      // init_guess.orientation.x = -offset_y_;
      // init_guess.orientation.y = -offset_y_;
      // init_guess.orientation.z = -offset_y_;
      // init_guess.orientation.w = -offset_y_;

      rclcpp::Rate rate(0.5);

      // Matching the contour once again
      contour_matching->setInitialGuess(init_guess);
      rate.sleep();

      // No need to match after setting the docking poses
      contour_matching->stopMatching();
      rate.sleep();

      if (!freeze_final_docking_pose()) {
        dock_poses_.clear();
        on_process_ = false;
        nav_task_finished_ = false;
        return;
      }

      dock_poses_.clear();

      // Look for and set the docking poses
      lookTransforms();

      startWaypointFollowing(dock_poses_);
    }
  }

  void result_callback(const WaypointFollowerGoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      dock_poses_.clear();
      nav_task_finished_ = true;
      // make_transforms();
      start_final_approach();
    }
  }

  // Broadcasting static transforms for the different poses involved in docking
  void make_transforms()
  {
    // Broadcast static tf pose for the first pre-dock position.
    geometry_msgs::msg::TransformStamped t1;

    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = docking_station_;
    t1.child_frame_id = "pre_dock";

    t1.transform.translation.x = (-1.8 + offset_x_);
    t1.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t1);

    // Broadcast static tf pose for the second pre-dock position.
    geometry_msgs::msg::TransformStamped t2;

    t2.header.stamp = this->get_clock()->now();
    t2.header.frame_id = docking_station_;
    t2.child_frame_id = "pre_dock2";

    t2.transform.translation.x = -pre_dock_dist_ ;
    t2.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(t2);
  }

  // Converts Transformpose to PoseStamped
  geometry_msgs::msg::PoseStamped ConvertTransformToPose(
    geometry_msgs::msg::TransformStamped & transform_pose)
  {
    geometry_msgs::msg::PoseStamped convert_pose;
    convert_pose.header = transform_pose.header;
    convert_pose.pose.position.x = transform_pose.transform.translation.x;
    convert_pose.pose.position.y = transform_pose.transform.translation.y;
    convert_pose.pose.position.z = 0.0;
    convert_pose.pose.orientation.x = transform_pose.transform.rotation.x;
    convert_pose.pose.orientation.y = transform_pose.transform.rotation.y;
    convert_pose.pose.orientation.z = transform_pose.transform.rotation.z;
    convert_pose.pose.orientation.w = transform_pose.transform.rotation.w;
    return convert_pose;
  }

  void
  startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
  {
    auto is_action_server_ready =
      waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) {
      RCLCPP_ERROR(
        client_node_->get_logger(), "follow_waypoints action server is not available."
        " Is the initial pose set?");
      return;
    }

    // Send the goal poses
    waypoint_follower_goal_.poses = poses;

    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
  }

  void
  goToPredock(geometry_msgs::msg::PoseStamped pose)
  {
    auto is_action_server_ready =
      navigate_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) {
      RCLCPP_ERROR(
        client_node_->get_logger(), "follow_waypoints action server is not available."
        " Is the initial pose set?");
      return;
    }

    // Send the goal poses
    nav_to_pos_goal_.pose = pose;

    navigate_to_pose_action_client_->async_send_goal(nav_to_pos_goal_, nav_to_goal_options);
  }

  void lookTransforms() {
    /** Couple of variables to store the robot, pre-dock
     * and docking station positions in the map **/
    geometry_msgs::msg::TransformStamped tempTransform;
    geometry_msgs::msg::TransformStamped robot_pose;

    try {
      robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and base_footprint" << std::endl;
      return;
    }

    // stage 1
    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return;
    }

    geometry_msgs::msg::PoseStamped pre_dock_pose = ConvertTransformToPose(tempTransform);
    
    dock_poses_.emplace_back(pre_dock_pose);

    // stage 2
    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock2", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and pre_dock2" << std::endl;
      dock_poses_.clear();
      return;
    }

    geometry_msgs::msg::PoseStamped pre_dock2_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(pre_dock2_pose);

    // stage 3
    try {
      tempTransform = buffer_->lookupTransform("map", docking_station_, tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and docking_link" << std::endl;
      dock_poses_.clear();
      return;
    }

    // Check if the robot is in the docking position, if so do nothing
    if (euclidean_distance(tempTransform, robot_pose) < 0.05) {

      RCLCPP_ERROR(this->get_logger(), "Still in the docking position, call undock");
      on_process_ = false;
      dock_poses_.clear();
    }
  }

  bool dock(
    std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    // clear the stored poses
    if (on_process_) {
      RCLCPP_ERROR(this->get_logger(), "Wait for the process to complete");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting to dock");

    // Never allow a new docking run to reuse a target from an earlier run.
    frozen_docking_pose_in_odom_.reset();
    on_process_ = true;
    
    contour_matching->stopMatching();

    // Check and set the docking poses
    lookTransforms();
    if (dock_poses_.empty()){
      on_process_ = false;
      return false;
    }

    if (!helper_set_safety_field(docking_safety_field_)) {
      RCLCPP_ERROR(this->get_logger(), "Cannot dock without safety field 8");
      on_process_ = false;
      dock_poses_.clear();
      return false;
    }

    goToPredock(dock_poses_[0]);
    dock_poses_.clear();
    return true;
  }

  bool undock(
    std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    if (on_process_) {
      RCLCPP_ERROR(this->get_logger(), "Wait for the process to complete");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting to undock");
    rclcpp::Rate loop_rate(100);
    constexpr int no_progress_timeout_seconds = 100;
    const auto no_progress_timeout =
      rclcpp::Duration::from_seconds(static_cast<double>(no_progress_timeout_seconds));
    constexpr double minimum_progress = 0.001;  // metres
    auto last_progress_time = this->get_clock()->now();
    double last_progress_distance = 0.0;
    bool timed_out = false;

    if (!helper_set_safety_field(docking_safety_field_)) {
      RCLCPP_ERROR(this->get_logger(), "Cannot undock without safety field 8");
      return false;
    }
    last_progress_time = this->get_clock()->now();

    on_process_ = true;

    /** Couple of variables to store the robot, pre-dock
     * and docking station positions in the map **/

    geometry_msgs::msg::TransformStamped robot_pose;
    double distance = 0.0;

    try {
      robot_pose = buffer_->lookupTransform("map", base_link_, tf2::TimePointZero);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "No transform found between map and %s: %s",
        base_link_.c_str(), ex.what());
      on_process_ = false;
      return false;
    }

    auto robot_docked_pose = robot_pose;
    geometry_msgs::msg::Twist twist_vel;
    rclcpp::Rate sleep_rate(0.5);

    while (distance < undock_dist_) {
      try {
        robot_pose = buffer_->lookupTransform("map", base_link_, tf2::TimePointZero);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          this->get_logger(), "No transform found between map and %s: %s",
          base_link_.c_str(), ex.what());
        geometry_msgs::msg::Twist stop_twist;
        vel_pub->publish(stop_twist);
        on_process_ = false;
        return false;
      }
      distance = euclidean_distance(robot_docked_pose, robot_pose);

      const auto now = this->get_clock()->now();
      if (distance >= last_progress_distance + minimum_progress) {
        last_progress_distance = distance;
        last_progress_time = now;
      } else if (now - last_progress_time >= no_progress_timeout) {
        timed_out = true;
        break;
      }

      twist_vel.linear.x = -0.1;

      vel_pub->publish(twist_vel);
      loop_rate.sleep();
    }

    // Setting 0 velocity
    twist_vel.linear.x = 0.0;
    vel_pub->publish(twist_vel);

    // Process finished
    on_process_ = false;

    if (timed_out) {
      RCLCPP_WARN(
        this->get_logger(),
        "Undocking timeout: robot made no progress for %d seconds",
        no_progress_timeout_seconds);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Undocking finished");

    if (!helper_set_safety_field(normal_safety_field_)) {
      RCLCPP_ERROR(this->get_logger(), "Undocked, but failed to restore safety field 0");
      return false;
    }

    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "Restored safety field 0");

    // Restart contour matching.
    geometry_msgs::msg::Pose init_pose;
    contour_matching->setInitialGuess(init_pose);
    
    return true;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;

  nav2_msgs::action::NavigateToPose::Goal nav_to_pos_goal_;

  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
    navigate_to_pose_action_client_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr docking_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr undocking_srv_;

  rclcpp::Client<neo_srvs2::srv::SetSafetyField>::SharedPtr set_safety_field_client_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<ContourMatching> contour_matching;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::vector<geometry_msgs::msg::PoseStamped> dock_poses_;
  std::optional<geometry_msgs::msg::TransformStamped> frozen_docking_pose_in_odom_;

  // extra node for docking client - for spinning multiple threads
  std::shared_ptr<rclcpp::Node> client_node_;

  bool on_process_ = false;
  bool nav_task_finished_ = false;
  bool set_approaching_ = false;
  bool goal_reached_ = false;

  static constexpr uint32_t docking_safety_field_ = 8;
  static constexpr uint32_t normal_safety_field_ = 0;

  std::string scan_topic_ = "scan";
  std::string final_approach_frame_ = "odom";
  std::string docking_station_ = "docking_link";
  std::string base_link_ = "base_footprint";

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  double offset_x_ = 0.0;
  double offset_y_ = 0.0;
  double offset_yaw_ = 0.0;
  double undock_dist_ = 0.0;
  double pre_dock_dist_ = 0.0;
  double approach_distance_ = 0.37;
  double distance_tolerance_ = 0.005;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoDocking>();

  if (!rclcpp::ok()) {
    return 1;
  }

  // multiple callback groups means multithreaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.add_node(nh->safety_client_node_);
  executor.spin();

  return 0;
}

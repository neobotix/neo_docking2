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

#include <fstream>
#include <chrono>
#include <cstdio>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

  NeoDocking()
  : Node("neo_docking2")
  {
    this->declare_parameter<std::vector<double>>("pose", {-1, 0, 0});
    this->declare_parameter<std::vector<double>>("orientation", {0, 0, 0.707, 0.707});
    this->declare_parameter<double>("laser_ref", 0.32);

    this->get_parameter("pose", pose_array_);
    this->get_parameter("orientation", orientation_array_);
    this->get_parameter("laser_ref", laser_ref_);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
    dock_poses_.reserve(2);

    // Seperate callback group for laserscan subscription
    sub_cb_grp_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = sub_cb_grp_;

    // call to dock
    docking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "go_and_dock", std::bind(&NeoDocking::dock, this, _1, _2));
    // call to undock
    undocking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "undock_and_arm", std::bind(&NeoDocking::undock, this, _1, _2));
    // call to store poses
    store_pose_srv_ = this->create_service<std_srvs::srv::Empty>(
      "store_pose", std::bind(&NeoDocking::store_pose, this, _1, _2));

    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    client_node_ = std::make_shared<rclcpp::Node>("docking_client_node");

    sensor_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar_1/scan_filtered", 10, std::bind(&NeoDocking::scan_callback, this, _1),
      options);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    waypoint_follower_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      client_node_,
      "follow_waypoints");

    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    // Seperate thread for recieving the result_callback
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NeoDocking::helper_thread, this));

    send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  }

  void helper_thread()
  {
    // Only have the result callback enabled if the control is not in the stage
    if (!nav_task_finished_) {
      send_goal_options.result_callback =
        std::bind(&NeoDocking::result_callback, this, _1);
    } else {
      // Stage 3 of docking
      geometry_msgs::msg::TransformStamped robot_pose;
      geometry_msgs::msg::TransformStamped checkTransform;

      try {
        robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      } catch (const std::exception & ex) {
        std::cout << "no trasformation found between map and base_footprint" << std::endl;
        return;
      }

      try {
        checkTransform = buffer_->lookupTransform("map", "docking_station", tf2::TimePointZero);
      } catch (const std::exception & ex) {
        std::cout << "no trasformation found between map and docking_station" << std::endl;
        return;
      }

      // determine the distance of the robot from docking station
      double distance = euclidean_distance(robot_pose, checkTransform);

      // additionaly layer check if docking has completed
      if (distance <= 0.015) {
        RCLCPP_INFO(client_node_->get_logger(), "Docking finished");
        on_process_ = false;
        nav_task_finished_ = false;
        return;
      }

      auto robot_docking_pose = checkTransform;
      geometry_msgs::msg::Twist twist_vel;

      /** setting conditions for the robot to dock
       * distance between the robot and docking station will vary
       * depending on the localization. Therefore, using laser-
       * reference to halt the robot **/
      if (distance > 0.015 && laser_ref_ < store_laser_ref_) {
        try {
          robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const std::exception & ex) {
          std::cout << "no trasformation found between map and base_footprint" << std::endl;
          return;
        }
        // Todo: Set the P-Gain from the ROS parameter server
        twist_vel.linear.x = distance * 0.50;
        vel_pub->publish(twist_vel);
      } else {
        RCLCPP_INFO(client_node_->get_logger(), "Docking finished");
        twist_vel.linear.x = 0.0;   // Setting 0 velocity
        vel_pub->publish(twist_vel);
        on_process_ = false;
        nav_task_finished_ = false;
      }
    }
    rclcpp::spin_some(client_node_);
  }

private:
  inline double euclidean_distance(
    geometry_msgs::msg::TransformStamped & pos1,
    const geometry_msgs::msg::TransformStamped & pos2)
  {
    double dx = pos1.transform.translation.x - pos2.transform.translation.x;
    double dy = pos1.transform.translation.y - pos2.transform.translation.y;

    return std::hypot(dx, dy);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr sensor_data)
  {
    auto data = sensor_data;
    store_laser_ref_ = data->ranges[static_cast<int>(data->ranges.size()) / 2];
  }

  void result_callback(const WaypointFollowerGoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      dock_poses_.clear();
      nav_task_finished_ = true;
    }
  }

  // Broadcasting static transforms for the different poses involved in docking
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    // 1. Broadcast static tf pose for the exact position of the docking station
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "docking_station";

    t.transform.translation.x = pose_array_[0];
    t.transform.translation.y = pose_array_[1];
    t.transform.translation.z = pose_array_[2];

    t.transform.rotation.x = orientation_array_[1];
    t.transform.rotation.y = orientation_array_[2];
    t.transform.rotation.z = orientation_array_[3];
    t.transform.rotation.w = orientation_array_[0];

    tf_static_broadcaster_->sendTransform(t);

    // 2. Broadcast static tf pose for the pre-pose of the docking station
    geometry_msgs::msg::TransformStamped t1;

    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = "docking_station";
    t1.child_frame_id = "pre_dock";

    t1.transform.translation.x = -0.5;
    t1.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t1);

    // 3. Broadcast static tf pose for the pre-pose2 of the docking station
    geometry_msgs::msg::TransformStamped t2;

    t2.header.stamp = this->get_clock()->now();
    t2.header.frame_id = "docking_station";
    t2.child_frame_id = "pre_dock2";

    t2.transform.translation.x = -0.30;
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

    auto future_goal_handle =
      waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);

    // spin the client node to check if the goal has reached
    if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
      return;
    }

    // Get the goal handle and save so that we can check on completion in the timer callback
    waypoint_follower_goal_handle_ = future_goal_handle.get();
    if (!waypoint_follower_goal_handle_) {
      RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server, docking failed");
      return;
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

    on_process_ = true;

    /** Couple of variables to store the robot, pre-dock
     * and docking station positions in the map **/
    geometry_msgs::msg::TransformStamped tempTransform;
    geometry_msgs::msg::TransformStamped robot_pose;

    try {
      robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and base_footprint" << std::endl;
      return false;
    }

    // stage 1
    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pre_dock_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(pre_dock_pose);

    // stage 2
    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock2", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pre_dock2_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(pre_dock2_pose);

    // Check if the robot is in the docking position, if so do nothing
    if (euclidean_distance(tempTransform, robot_pose) < 0.05) {
      RCLCPP_ERROR(this->get_logger(), "Still in the docking position");
      on_process_ = false;
      dock_poses_.clear();
      return false;
    }

    startWaypointFollowing(dock_poses_);

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

    on_process_ = true;

    /** Couple of variables to store the robot, pre-dock
     * and docking station positions in the map **/

    geometry_msgs::msg::TransformStamped robot_pose;
    geometry_msgs::msg::TransformStamped checkTransform;

    double distance = 0.0;

    try {
      robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and base_footprint" << std::endl;
      return false;
    }

    try {
      checkTransform = buffer_->lookupTransform("map", "docking_station", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    if (euclidean_distance(robot_pose, checkTransform) > 0.07) {
      RCLCPP_ERROR(this->get_logger(), "Not in the docking position");
      on_process_ = false;
      return false;
    }

    auto robot_docked_pose = robot_pose;
    geometry_msgs::msg::Twist twist_vel;

    while (distance < 0.5) {
      try {
        robot_pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      } catch (const std::exception & ex) {
        std::cout << "no trasformation found between map and base_footprint" << std::endl;
        return false;
      }
      distance = euclidean_distance(robot_docked_pose, robot_pose);
      twist_vel.linear.x = -0.1;

      vel_pub->publish(twist_vel);
    }

    // Setting 0 velocity
    twist_vel.linear.x = 0.0;
    vel_pub->publish(twist_vel);

    // Process finished
    on_process_ = false;
    RCLCPP_INFO(client_node_->get_logger(), "Undocking finished");

    return true;
  }

  bool store_pose(
    std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    if (on_process_) {
      RCLCPP_ERROR(this->get_logger(), "Already process started, cannot update the pose");
      return false;
    }

    geometry_msgs::msg::TransformStamped tempTransform;

    try {
      tempTransform = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const std::exception & ex) {
      std::cout << "no trasformation found between map and base_footprint" << std::endl;
      return false;
    }

    auto robot_pose = ConvertTransformToPose(tempTransform);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "neo_docking2";
    out << YAML::BeginMap;

    // Save WPs to data structure
    out << YAML::Key << "ros__parameters";
    out << YAML::BeginMap;
    out << YAML::Key << "pose";
    std::vector<double> pose =
    {robot_pose.pose.position.x, robot_pose.pose.position.y,
      robot_pose.pose.position.z};
    out << YAML::Value << pose;
    out << YAML::Key << "orientation";
    std::vector<double> orientation =
    {robot_pose.pose.orientation.w, robot_pose.pose.orientation.x,
      robot_pose.pose.orientation.y, robot_pose.pose.orientation.z};
    out << YAML::Value << orientation;
    out << YAML::Key << "laser_ref";
    out << YAML::Value << store_laser_ref_;
    out << YAML::EndMap;

    std::ofstream fout("src/neo_docking2/launch/dock_pose.yaml");
    fout << out.c_str();

    RCLCPP_INFO(client_node_->get_logger(), "Poses stored");

    return true;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::vector<double> pose_array_;
  std::vector<double> orientation_array_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;

  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr docking_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr undocking_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr store_pose_srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sensor_sub;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::vector<geometry_msgs::msg::PoseStamped> dock_poses_;

  // extra node for docking client - for spinning multiple threads
  std::shared_ptr<rclcpp::Node> client_node_;

  bool on_process_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  rclcpp::CallbackGroup::SharedPtr sub_cb_grp_;
  rclcpp::SubscriptionOptions options;
  bool nav_task_finished_ = false;

  double laser_ref_ = 0.0;
  double store_laser_ref_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoDocking>();

  // multiple callback groups means multithreaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.spin();

  return 0;
}

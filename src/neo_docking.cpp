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

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class NeoDocking : public rclcpp::Node
{
public:
  NeoDocking() : Node("neo_docking") 
  {
    this->declare_parameter<std::vector<double>>("Pose", {-1, 0, 0});
    this->declare_parameter<std::vector<double>>("orientation", {0, 0, 0.707, 0.707});

    this->get_parameter("Pose", pose_array_);
    this->get_parameter("orientation", orientation_array_);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
    dock_poses_.reserve(2);

    docking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "go_and_dock", std::bind(&NeoDocking::dock, this, _1, _2));
    undocking_srv_ = this->create_service<std_srvs::srv::Empty>(
      "undock_and_arm", std::bind(&NeoDocking::undock, this, _1, _2));

    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    client_node_ = std::make_shared<rclcpp::Node>("docking_client_node");

    waypoint_follower_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      client_node_,
      "follow_waypoints");

    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
  }

private:
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "docking_station";

    t.transform.translation.x = pose_array_[0];
    t.transform.translation.y = pose_array_[1];
    t.transform.translation.z = pose_array_[2];

    t.transform.rotation.x = orientation_array_[0];
    t.transform.rotation.y = orientation_array_[1];
    t.transform.rotation.z = orientation_array_[2];
    t.transform.rotation.w = orientation_array_[3];

    tf_static_broadcaster_->sendTransform(t);

    geometry_msgs::msg::TransformStamped t1;

    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = "docking_station";
    t1.child_frame_id = "pre_dock";

    t1.transform.translation.x = -0.5;
    t1.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t1);
  }

  geometry_msgs::msg::PoseStamped ConvertTransformToPose(
    geometry_msgs::msg::TransformStamped transform_pose)
  {
    geometry_msgs::msg::PoseStamped convert_pose;
    convert_pose.header = transform_pose.header;
    convert_pose.pose.position.x = transform_pose.transform.translation.x;
    convert_pose.pose.position.y = transform_pose.transform.translation.y;
    convert_pose.pose.position.z = transform_pose.transform.translation.z;
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

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

    send_goal_options.result_callback = [this](WaypointFollowerGoalHandle::WrappedResult result) {
        waypoint_follower_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          on_process_ = false;
        }
      };

    auto future_goal_handle =
      waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
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

  bool dock(std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    // clear the stored poses
    if (on_process_) {
      RCLCPP_ERROR(this->get_logger(), "Wait for the process to complete");
      return false;
    }

    on_process_ = true;
    dock_poses_.clear();
    geometry_msgs::msg::TransformStamped tempTransform;

    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock", tf2::TimePointZero);  
    } catch(const std::exception& ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pre_dock_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(pre_dock_pose);

    try {
      tempTransform = buffer_->lookupTransform("map", "docking_station", tf2::TimePointZero);  
    } catch(const std::exception& ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped dock_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(dock_pose);

    startWaypointFollowing(dock_poses_);

    return true;
  }

  bool undock(std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    if (on_process_) {
      RCLCPP_ERROR(this->get_logger(), "Wait for the process to complete");
      return false;
    }

    on_process_ = true;
    // clear the stored poses
    dock_poses_.clear();

    geometry_msgs::msg::TransformStamped tempTransform;

    try {
      tempTransform = buffer_->lookupTransform("map", "pre_dock", tf2::TimePointZero);  
    } catch(const std::exception& ex) {
      std::cout << "no trasformation found between map and pre_dock" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pre_dock_pose = ConvertTransformToPose(tempTransform);
    dock_poses_.emplace_back(pre_dock_pose);

    startWaypointFollowing(dock_poses_);
    return true;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::vector<double> pose_array_;
  std::vector<double> orientation_array_;
  
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr docking_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr undocking_srv_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::vector<geometry_msgs::msg::PoseStamped> dock_poses_;

  // extra node for docking client - for spinning multiple threads
  std::shared_ptr<rclcpp::Node> client_node_;

  bool on_process_ = false;

};

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoDocking>();
  rclcpp::spin(nh);

  return 0;
}

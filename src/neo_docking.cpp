#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include "tf2_ros/static_transform_broadcaster.h"

class NeoDocking : public rclcpp::Node
{
public:
  NeoDocking() : Node("neo_docking") 
  {
    this->declare_parameter<std::vector<double>>("Pose", {1, 0, 0});
    this->declare_parameter<std::vector<double>>("orientation", {0, 0, 0.707, 0.707});

    this->get_parameter("Pose", pose_array_);
    this->get_parameter("orientation", orientation_array_);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
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

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::vector<double> pose_array_;
  std::vector<double> orientation_array_;
};

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoDocking>();
  rclcpp::spin(nh);

  return 0;
}

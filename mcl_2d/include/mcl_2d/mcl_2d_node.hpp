#pragma once
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mcl_2d/msg_converter.hpp"
#include "mcl_2d/util.hpp"

class Mcl2dNode : public rclcpp::Node {
 public:
  Mcl2dNode();
  virtual ~Mcl2dNode() = default;
  void loop();

  int odom_freq_;

 private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void initTF();
  bool getOdomPose(Vector3d& pose);
  bool getLidarPose(Vector3d& pose);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription;

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr selfpose_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_mapped_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_publisher;

  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  tf2::Transform latest_tf_;
  rclcpp::Clock ros_clock_;
  sensor_msgs::msg::LaserScan::SharedPtr laser_msg;
  std::string global_frame_id_, footprint_frame_id_, odom_frame_id_, base_frame_id_, scan_frame_id_;
  rclcpp::Time scan_time_stamp_;
  double transform_tolerance_;

  MsgConverter msg_converter;
  Util util;
  Mcl2d mcl_2d;
};

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mcl_2d/msg_converter.hpp"
#include "mcl_2d/util.hpp"


class Mcl2dNode : public rclcpp::Node {
public:
  Mcl2dNode();
  virtual ~Mcl2dNode() = default;

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void broadcastTransforms();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_subscription;

  std::shared_ptr<tf2_ros::TransformBroadcaster>              broadcaster;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr   selfpose_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_mapped_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_publisher;

  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  Vector3d odom = Vector3d::Zero();
  Vector3d laser = Vector3d::Zero();

  MsgConverter msg_converter;
  Util util;
};


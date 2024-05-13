#pragma once

#include <cstring>  // for memcpy
#include <eigen3/Eigen/Dense>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mcl_2d/mcl_2d.hpp"
#include "mcl_2d/scan.hpp"

using namespace std;
using namespace Eigen;

class MsgConverter {
  public:
  geometry_msgs::msg::TransformStamped broadcastWorldToBaseLink(const double& x, const double& y, const double& yaw);
  geometry_msgs::msg::TransformStamped broadcastBaseLinkToLidarFrame(const std::vector<double>& tf_laser2robot);
  geometry_msgs::msg::PoseArray        createParticleCloud(std::vector<Mcl2d::Particle>& particles);
  sensor_msgs::msg::PointCloud2        createTransformedPC2(Eigen::Matrix4Xf& eigenLaser, Eigen::Matrix4f& transMatrix);

  vector<LaserPoint> scan_to_vector(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Vector3d &laser);
  sensor_msgs::msg::PointCloud2 vector_to_PC2(vector<LaserPoint> &points);

};
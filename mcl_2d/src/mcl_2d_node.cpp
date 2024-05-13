#include <cstdio>

#include "mcl_2d/mcl_2d_node.hpp"

Mcl2dNode::Mcl2dNode() : Node("mcl_2d_node") {
  laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Mcl2dNode::laser_callback, this, std::placeholders::_1));
  odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Mcl2dNode::odom_callback, this, std::placeholders::_1));

  selfpose_publisher   = this->create_publisher<geometry_msgs::msg::Vector3>("self_pose", _qos);
  pc2_mapped_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_mapped", _qos);
  particle_publisher   = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", _qos);
  broadcaster         = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void Mcl2dNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received scan");
  vector<LaserPoint> src_points = msg_converter.scan_to_vector(msg, odom);  //本来laserの位置
  sensor_msgs::msg::PointCloud2 cloud = msg_converter.vector_to_PC2(src_points);
  pc2_mapped_publisher->publish(cloud);
}

void Mcl2dNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received odom: position x: %f, y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  odom[0] = msg->pose.pose.position.x;
  odom[1] = msg->pose.pose.position.y;
  odom[2] = util.quaternionToYaw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  broadcastTransforms();
}

void Mcl2dNode::broadcastTransforms() {
  std::vector<double> tf_laser2robot = {1.0, 2.0, 0.0, 0.1, 0.2, 0.3};
  geometry_msgs::msg::TransformStamped world_to_base_link = msg_converter.broadcastWorldToBaseLink(odom[0], odom[1], odom[2]);
  geometry_msgs::msg::TransformStamped base_link_to_lidar = msg_converter.broadcastBaseLinkToLidarFrame(tf_laser2robot);
  broadcaster->sendTransform(world_to_base_link);
  broadcaster->sendTransform(base_link_to_lidar);
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mcl2dNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

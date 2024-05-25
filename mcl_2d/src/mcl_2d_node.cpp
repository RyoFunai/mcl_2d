#include "mcl_2d/mcl_2d_node.hpp"

#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace std;

Mcl2dNode::Mcl2dNode() : Node("mcl_2d"),
                         ros_clock_(RCL_SYSTEM_TIME) {
  laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Mcl2dNode::laser_callback, this, std::placeholders::_1));

  pc2_mapped_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_mapped", _qos);
  particle_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", _qos);
  broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);  // shared_from_this()はコンストラクタでは使えない。（オブジェクト化が完了していないから？）

  this->declare_parameter("global_frame_id", std::string("map"));
  this->declare_parameter("footprint_frame_id", std::string("base_footprint"));
  this->declare_parameter("odom_frame_id", std::string("odom"));
  this->declare_parameter("base_frame_id", std::string("base_link"));
  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  footprint_frame_id_ = this->get_parameter("footprint_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  this->declare_parameter("odom_freq", 20);
  this->get_parameter("odom_freq", odom_freq_);
  this->declare_parameter("initial_pose", std::vector<double>{0.0, 0.0, 0.0});
  const auto initial_pose_vec = this->get_parameter("initial_pose").as_double_array();
  this->declare_parameter("particles_num", 100);
  const auto particles_num_ = this->get_parameter("particles_num").as_int();
  this->declare_parameter("map_param_path", std::string(""));
  const auto map_param_path_ = this->get_parameter("map_param_path").as_string();

  Vector3d initial_pose_ = Vector3d::Zero();
  initial_pose_ << initial_pose_vec[0], initial_pose_vec[1], initial_pose_vec[2];

  mcl_2d.loadMap(map_param_path_);
  mcl_2d.init_particles(initial_pose_, particles_num_);
  initTF();
}

void Mcl2dNode::loop() {
  Vector3d odom = Vector3d::Zero();
  Vector3d laser = Vector3d::Zero();
  if (!getOdomPose(odom) || !getLidarPose(laser)) {
    RCLCPP_WARN(this->get_logger(), "odom x: %f, y: %f, yaw: %f", odom[0], odom[1], odom[2]);
    RCLCPP_WARN(this->get_logger(), "laser x: %f, y: %f, yaw: %f", laser[0], laser[1], laser[2]);
    return;
  }
  vector<LaserPoint> src_points = msg_converter.scan_to_vector(laser_msg, laser);

  sensor_msgs::msg::PointCloud2 cloud = msg_converter.vector_to_PC2(src_points);
  pc2_mapped_publisher->publish(cloud);

  geometry_msgs::msg::TransformStamped world_to_base_link = msg_converter.broadcastWorldToBaseLink(odom);
  geometry_msgs::msg::TransformStamped base_link_to_lidar = msg_converter.broadcastBaseLinkToLidarFrame(laser);
  tf_broadcaster->sendTransform(world_to_base_link);
  tf_broadcaster->sendTransform(base_link_to_lidar);
  publish_particle();
}

void Mcl2dNode::initTF() {
  tf_broadcaster.reset();
  tf_listener.reset();
  tf_.reset();
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface(),
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
  tf_->setCreateTimerInterface(timer_interface);
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_);
  // tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  // tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(std::enable_shared_from_this<Mcl2dNode>::shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
}

void Mcl2dNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received scan");
  laser_msg = msg;
  scan_frame_id_ = laser_msg->header.frame_id;
}

bool Mcl2dNode::getOdomPose(Vector3d& pose) {
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = footprint_frame_id_;
  ident.header.stamp = rclcpp::Time(0);
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  geometry_msgs::msg::PoseStamped odom_pose;
  try {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  } catch (tf2::TransformException& e) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  pose[0] = odom_pose.pose.position.x;
  pose[1] = odom_pose.pose.position.y;
  pose[2] = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

bool Mcl2dNode::getLidarPose(Vector3d& pose) {
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = scan_frame_id_;
  ident.header.stamp = ros_clock_.now();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  geometry_msgs::msg::PoseStamped lidar_pose;
  try {
    this->tf_->transform(ident, lidar_pose, base_frame_id_);
  } catch (tf2::TransformException& e) {
    RCLCPP_WARN(get_logger(), "Failed to compute lidar pose, skipping scan (%s)", e.what());
    return false;
  }

  pose[0] = lidar_pose.pose.position.x;
  pose[1] = lidar_pose.pose.position.y;
  pose[2] = tf2::getYaw(lidar_pose.pose.orientation);

  return true;
}

void Mcl2dNode::publish_particle() {
  std::vector<Mcl2d::Particle> particles = mcl_2d.getParticles();
  auto pose_array_msg = msg_converter.createParticleCloud(particles);
  particle_publisher->publish(pose_array_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mcl2dNode>();
  rclcpp::Rate loop_rate(node->odom_freq_);
  while (rclcpp::ok()) {
    node->loop();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
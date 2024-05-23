#include "mcl_2d/mcl_2d_node.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include <opencv2/opencv.hpp>

using namespace std;

Mcl2dNode::Mcl2dNode() : Node("mcl_2d"),
                         ros_clock_(RCL_SYSTEM_TIME) {
  laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Mcl2dNode::laser_callback, this, std::placeholders::_1));

  selfpose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("self_pose", _qos);
  pc2_mapped_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_mapped", _qos);
  particle_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", _qos);
  broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

	this->declare_parameter("global_frame_id", std::string("map"));
	this->declare_parameter("footprint_frame_id", std::string("base_footprint"));
	this->declare_parameter("odom_frame_id", std::string("odom"));
	this->declare_parameter("base_frame_id", std::string("base_link"));
	this->get_parameter("global_frame_id", global_frame_id_);
	this->get_parameter("footprint_frame_id", footprint_frame_id_);
	this->get_parameter("odom_frame_id", odom_frame_id_);
	this->get_parameter("base_frame_id", base_frame_id_);

	this->declare_parameter("odom_freq", 20);
	this->get_parameter("odom_freq", odom_freq_);

  loadMap("iscas_museum_map.yaml");

  initTF();
}

void Mcl2dNode::loop() {
  double x, y, yaw;
  if (getOdomPose(x, y, yaw) && getLidarPose(x, y, yaw)) {
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", x, y, yaw);
  }
}

void Mcl2dNode::loadMap(const std::string& yaml_path) {
  std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path map_dir = source_dir / "../../config/maps";
  YAML::Node config = YAML::LoadFile(map_dir.string() + "/" + yaml_path);
  std::filesystem::path image_path = map_dir / config["image"].as<std::string>();
  double resolution = config["resolution"].as<double>();
  std::vector<double> origin = config["origin"].as<std::vector<double>>();
  int negate = config["negate"].as<int>();
  double occupied_thresh = config["occupied_thresh"].as<double>();
  double free_thresh = config["free_thresh"].as<double>();

  cv::Mat map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (map_image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_path.c_str());
    return;
  }
  
  cv::imshow("Map", map_image);
  cv::waitKey(0);  // キー入力を待つ

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %s", image_path.c_str());
}


void Mcl2dNode::initTF() {
  tfb_.reset();
  tfl_.reset();
  tf_.reset();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface(),
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
  tf_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
}

void Mcl2dNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received scan");

	scan_frame_id_ = msg->header.frame_id;
  vector<LaserPoint> src_points = msg_converter.scan_to_vector(msg, odom);  // 本来laserの位置
  sensor_msgs::msg::PointCloud2 cloud = msg_converter.vector_to_PC2(src_points);
  pc2_mapped_publisher->publish(cloud);
}

bool Mcl2dNode::getOdomPose(double& x, double& y, double& yaw) {
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
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

bool Mcl2dNode::getLidarPose(double& x, double& y, double& yaw) {
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

  x = lidar_pose.pose.position.x;
  y = lidar_pose.pose.position.y;

  double roll, pitch;
  tf2::getEulerYPR(lidar_pose.pose.orientation, yaw, pitch, roll);

  return true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mcl2dNode>();
	rclcpp::Rate loop_rate(node->odom_freq_);
	while (rclcpp::ok()) {
		// node->loop();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
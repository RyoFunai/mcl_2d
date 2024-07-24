#include "mcl_2d/msg_converter.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>


geometry_msgs::msg::TransformStamped MsgConverter::broadcastWorldToBaseLink(const Vector3f& pose) {
  geometry_msgs::msg::TransformStamped world_to_base_link;

  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
  world_to_base_link.header.stamp = ros_clock.now();
  world_to_base_link.header.frame_id = "map";
  world_to_base_link.child_frame_id = "base_link_";
  // base_link の絶対位置と向きを設定
  world_to_base_link.transform.translation.x = pose.x();
  world_to_base_link.transform.translation.y = pose.y();
  world_to_base_link.transform.rotation.z = sin(pose.z() / 2.0);
  world_to_base_link.transform.rotation.w = cos(pose.z() / 2.0);

  return world_to_base_link;
}

geometry_msgs::msg::TransformStamped MsgConverter::broadcastBaseLinkToLidarFrame(const Vector3f& pose) {
  geometry_msgs::msg::TransformStamped base_link_to_lidar;

  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
  base_link_to_lidar.header.stamp = ros_clock.now();
  base_link_to_lidar.header.frame_id = "base_link_";
  base_link_to_lidar.child_frame_id = "lidar_frame_";

  // lidar_frame の base_link に対する相対位置と向きを設定
  base_link_to_lidar.transform.translation.x = pose.x();
  base_link_to_lidar.transform.translation.y = pose.y();
  base_link_to_lidar.transform.rotation.z = sin(pose.z() / 2.0);
  base_link_to_lidar.transform.rotation.w = cos(pose.z() / 2.0);

  return base_link_to_lidar;
}

geometry_msgs::msg::PoseArray MsgConverter::createParticleCloud(const std::vector<Particle>& particles) {
  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME); 

  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = ros_clock.now();
  pose_array_msg.header.frame_id = "map";

  // パーティクルをPoseArrayに追加
  for (const auto& particle : particles) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particle.pose(0, 2);
    pose.position.y = particle.pose(1, 2);
    pose.position.z = 0.0;

    // パーティクルの姿勢を四元数に変換
    Eigen::Matrix2f rotation = particle.pose.block<2, 2>(0, 0);
    Eigen::Rotation2Df rot2d(rotation);                                                // 2D回転行列から回転角度を取得
    Eigen::Quaternionf q(Eigen::AngleAxisf(rot2d.angle(), Eigen::Vector3f::UnitZ()));  // Z軸周りの回転として四元数を作成
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_array_msg.poses.push_back(pose);
  }

  return pose_array_msg;
}



visualization_msgs::msg::MarkerArray MsgConverter::createParticleCloudMarkerArray(const std::vector<Particle>& particles) {
  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

  visualization_msgs::msg::MarkerArray marker_array_msg;

  // パーティクルの最大スコアと最小スコアを見つける
  float max_score = std::numeric_limits<float>::lowest();
  float min_score = std::numeric_limits<float>::max();
  for (const auto& particle : particles) {
    max_score = std::max(max_score, particle.score);
    min_score = std::min(min_score, particle.score);
  }

  // パーティクルの最大スコアを見つける
  for (const auto& particle : particles) {
    if (particle.score > max_score) {
      max_score = particle.score;
    }
  }
  float score_range = max_score - min_score;


  // パーティクルをMarkerArrayに追加
  for (size_t i = 0; i < particles.size(); ++i) {
    const auto& particle = particles[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros_clock.now();
    marker.ns = "particles";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = particle.pose(0, 2);
    marker.pose.position.y = particle.pose(1, 2);
    marker.pose.position.z = 0.0;

    Eigen::Matrix2f rotation = particle.pose.block<2, 2>(0, 0);
    Eigen::Rotation2Df rot2d(rotation);
    Eigen::Quaternionf q(Eigen::AngleAxisf(rot2d.angle(), Eigen::Vector3f::UnitZ()));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 0.2;  // 矢印の長さ
    marker.scale.y = 0.02; // 矢印の幅
    marker.scale.z = 0.02; // 矢印の高さ

    // float normalized_score = particle.score / max_score;
    float normalized_score;
    if (score_range > 0) {
      normalized_score = (particle.score - min_score) / score_range;
    } else {
      // すべてのスコアが同じ場合
      normalized_score = 1.0;
    }

    marker.color.r = normalized_score;
    marker.color.g = 1.0f - normalized_score;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array_msg.markers.push_back(marker);
  }

  return marker_array_msg;
}

sensor_msgs::msg::PointCloud2 MsgConverter::createTransformedPC2(Eigen::Matrix4Xf& eigenLaser, Eigen::Matrix4f& transMatrix) {
  // マップ座標系への変換を行う
  Eigen::Matrix4Xf map_frame_points = transMatrix * eigenLaser;

  // PointCloud2メッセージを作成
  sensor_msgs::msg::PointCloud2 cloud_msg;
  // cloud_msg.header.stamp    = this->now();  // 現在の時刻を設定
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = map_frame_points.cols();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i) {
    cloud_msg.fields[i].offset = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  // ポイントデータをPointCloud2メッセージに追加
  for (int i = 0; i < cloud_msg.width; ++i) {
    memcpy(&cloud_msg.data[i * cloud_msg.point_step], &map_frame_points(0, i), sizeof(float) * 3);
  }

  return cloud_msg;
}

vector<LaserPoint> MsgConverter::scan_to_vector(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  vector<LaserPoint> src_points;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    LaserPoint src_point;
    if (msg->ranges[i] > 12 || msg->ranges[i] < 0.5) continue;
    src_point.x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);  // 極座標
    src_point.y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
    src_points.push_back(src_point);
  }
  return src_points;
}

vector<LaserPoint> MsgConverter::scan_to_vector(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Vector3f& laser) {
  vector<LaserPoint> src_points;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    LaserPoint src_point;
    if (msg->ranges[i] > 12 || msg->ranges[i] < 0.5) continue;
    double x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);  // 極座標
    double y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
    src_point.x = x * cos(laser[2]) - y * sin(laser[2]) + laser[0];  // 直交座標
    src_point.y = x * sin(laser[2]) + y * cos(laser[2]) + laser[1];
    src_points.push_back(src_point);
  }
  return src_points;
}

sensor_msgs::msg::PointCloud2 MsgConverter::vector_to_PC2(vector<LaserPoint>& points) {
  vector<vector<float>> points_;
  vector<float> point;
  for (size_t i = 0; i < points.size(); i++) {
    point = {static_cast<float>(points[i].x), static_cast<float>(points[i].y), 0};
    points_.push_back(point);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = points_.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i) {
    cloud_msg.fields[i].offset = i * 4;
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 4 * 3;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  // ポイントデータをPointCloud2メッセージに追加
  int j = 0;
  for (auto& point : points_) {
    memcpy(&cloud_msg.data[j], &point[0], sizeof(float) * 3);
    j += cloud_msg.point_step;
  }

  return cloud_msg;
}
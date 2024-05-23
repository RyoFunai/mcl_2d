#include "mcl_2d/msg_converter.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_array.hpp>


geometry_msgs::msg::PoseArray MsgConverter::createParticleCloud(std::vector<Mcl2d::Particle> &particles) {
  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header.stamp    = ros_clock.now();
  pose_array_msg.header.frame_id = "map";

  // パーティクルをPoseArrayに追加
  for (const auto& particle : particles) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particle.pose(0, 3);
    pose.position.y = particle.pose(1, 3);
    pose.position.z = 0.0;

    // パーティクルの姿勢を四元数に変換
    Eigen::Matrix3f    rotation = particle.pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_array_msg.poses.push_back(pose);
  }

  return pose_array_msg;
}

sensor_msgs::msg::PointCloud2 MsgConverter::createTransformedPC2(Eigen::Matrix4Xf& eigenLaser, Eigen::Matrix4f& transMatrix) {
  // マップ座標系への変換を行う
  Eigen::Matrix4Xf map_frame_points = transMatrix * eigenLaser;

  // PointCloud2メッセージを作成
  sensor_msgs::msg::PointCloud2 cloud_msg;
  // cloud_msg.header.stamp    = this->now();  // 現在の時刻を設定
  cloud_msg.header.frame_id = "map";
  cloud_msg.height          = 1;
  cloud_msg.width           = map_frame_points.cols();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i) {
    cloud_msg.fields[i].offset   = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count    = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense     = true;

  // ポイントデータをPointCloud2メッセージに追加
  for (int i = 0; i < cloud_msg.width; ++i) {
    memcpy(&cloud_msg.data[i * cloud_msg.point_step], &map_frame_points(0, i), sizeof(float) * 3);
  }

  return cloud_msg;
}

  vector<LaserPoint> MsgConverter::scan_to_vector(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Vector3d &laser){
    vector<LaserPoint> src_points;
    for(size_t i=0; i< msg->ranges.size(); ++i) {
      LaserPoint src_point;
      if(msg->ranges[i] > 12 || msg->ranges[i] < 0.5) continue;
        double x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);//極座標
        double y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
        src_point.x = x * cos(laser[2]) - y * sin(laser[2]) + laser[0];//直交座標
        src_point.y = x * sin(laser[2]) + y * cos(laser[2]) + laser[1];
        src_points.push_back(src_point);
    }
    return src_points;
  }

  sensor_msgs::msg::PointCloud2 MsgConverter::vector_to_PC2(vector<LaserPoint> &points){
    vector<vector<float>> points_;
    vector<float> point;
    for(size_t i=0; i<points.size(); i++){
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

    for (int i = 0; i < 3; ++i){
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
    for (auto& point : points_){
      memcpy(&cloud_msg.data[j], &point[0], sizeof(float) * 3);
      j += cloud_msg.point_step;
    }

    return cloud_msg;
  }
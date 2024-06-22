#include "mcl_2d/util.hpp"
#include <cmath>

double Util::quaternionToYaw(const double& w, const double& x, const double& y, const double& z) {
    double roll, pitch, yaw;
    // asinの範囲を超えないようにクランプ
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}

std::vector<LaserPoint> Util::transformToMapCoordinates(const std::vector<LaserPoint>& src_points, const Vector3f& pose) {
  std::vector<LaserPoint> map_points;
  Eigen::Rotation2Df rotation(pose.z());  // Z軸周りの回転行列を生成
  Eigen::Matrix2f rotation_matrix = rotation.toRotationMatrix();

  for (const auto& point : src_points) {
    Eigen::Vector2f laser_point(point.x, point.y);  // 2次元に変更
    Eigen::Vector2f rotated_point = rotation_matrix * laser_point;  // 回転を適用
    LaserPoint map_point;
    map_point.x = rotated_point.x() + pose.x();
    map_point.y = rotated_point.y() + pose.y();
    map_points.push_back(map_point);
  }
  return map_points;
}
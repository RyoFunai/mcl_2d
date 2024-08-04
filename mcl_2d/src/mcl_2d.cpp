#include "mcl_2d/mcl_2d.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <random>


void Mcl2d::setup(const string& yaml_path, const vector<double>& odom_convariance) {
  odom_convariance_ = odom_convariance;
  loadMap(yaml_path);
}

void Mcl2d::loadMap(const string& yaml_path) {
  filesystem::path yaml_file_path = yaml_path;
  YAML::Node config = YAML::LoadFile(yaml_file_path.string());
  filesystem::path image_path = yaml_file_path.parent_path() / config["likelihood_image"].as<string>();
  image_resolution = config["resolution"].as<double>();
  vector<double> origin = config["origin"].as<vector<double>>();
  map_origin << origin[0], origin[1], origin[2];
  // int negate = config["negate"].as<int>();
  // double occupied_thresh = config["occupied_thresh"].as<double>();
  // double free_thresh = config["free_thresh"].as<double>();

  map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

  if (map_image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Failed to load map image: %s", image_path.c_str());
    exit(0);
  }

  cv::flip(map_image, map_image, 0);

  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Map loaded successfully: %s", image_path.c_str());

  // ウィンドウを作成
  // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  // // マウスコールバックを設定
  // cv::setMouseCallback("Image", onMouse, &map_image);

  // // 画像を表示
  // cv::imshow("Image", map_image);
  // cv::waitKey(0);  // キー入力を待つ
  // exit(0);
}

Vector3f Mcl2d::updateData(const Vector3f& pose, const vector<LaserPoint>& src_points, const int particles_num) {
  diff_pose = pose - pre_pose;
  diff_pose.z() = util.normalizeAngle(diff_pose.z());
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "pose     : %f, %f, %f", pose.x(), pose.y(), pose.z());
  // RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "pre_pose : %f, %f, %f", pre_pose.x(), pre_pose.y(), pre_pose.z());
  // RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "diff_pose: %f, %f, %f", diff_pose.x(), diff_pose.y(), diff_pose.z());
  auto start = std::chrono::high_resolution_clock::now();  // 計測開始

  if (is_first_time) {
    particles = particle_sampler->generate_particles_normal(pose, particles_num);
    is_first_time = false;
  } else {
    particle_sampler->sampling(diff_pose, particles);
  }

  // particles = particle_sampler->generate_particles_normal(pose, particles_num);

  likelihood(src_points, particles);
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "updateData function took %lld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());

  if (particle_sampler->normalizeBelief(particles) > 1.0E-6) {
    particle_sampler->systematic_resample(particles);
  } else {
    // resetWeight();
    RCLCPP_WARN(rclcpp::get_logger("mcl_2d"), "No particle with positive score");
  }
  Vector3f current_pose = estimate_current_pose(particles);
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "current_pose: %f, %f, %f", current_pose.x(), current_pose.y(), current_pose.z());

  Vector3f est_diff_pose = current_pose - pose;
  est_diff_pose.z() = util.normalizeAngle(est_diff_pose.z());
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "est_diff_pose: %f, %f, %f", est_diff_pose.x(), est_diff_pose.y(), est_diff_pose.z());

  pre_pose = current_pose;

  is_first_time = false;
  return current_pose;
}


void Mcl2d::likelihood(const vector<LaserPoint>& src_points, vector<Particle>& particles) {
  float max_score = 0;
  for (auto& particle : particles) {
    float weight = 0;

    for (auto& point : src_points) {
      Eigen::Vector2f laser_point(point.x, point.y);
      Eigen::Rotation2Df rotation(particle.pose.z());
      Eigen::Vector2f map_point = rotation * laser_point + particle.pose.head<2>();
      auto [image_pt_x, image_pt_y] = mapToPixel(map_point.x(), map_point.y());

      if (image_pt_x < 0 || image_pt_x >= map_image.cols || image_pt_y < 0 || image_pt_y >= map_image.rows)
        continue;
      else {
        double pixel_val = (255 - map_image.at<uchar>(image_pt_y, image_pt_x)) / (double)255;
        weight += pixel_val;
      }
    }
    particle.score += weight / src_points.size();
    if (max_score < particle.score) {
      highest_weight_particle = particle;
      max_score = particle.score;
    }
  }
}

Vector3f Mcl2d::estimate_current_pose(const vector<Particle>& particles) {
  Vector3f pose = Vector3f::Zero();
  double total_weight = 0.0;
  double cos_sum = 0.0;
  double sin_sum = 0.0;
  double normalized_angle = 0.0;

  for (const auto& particle : particles) {
    double weight = particle.score;
    total_weight += weight;

    // 位置の重み付き和を計算
    pose.x() += particle.pose.x() * weight;
    pose.y() += particle.pose.y() * weight;
    pose.z() += particle.pose.z() * weight;
    normalized_angle += util.normalizeAngle(particle.pose.z() + M_PI) * weight;
  }

  if (total_weight > 0) {
    pose.x() /= total_weight;
    pose.y() /= total_weight;
    pose.z() /= total_weight;
    normalized_angle /= total_weight;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Total weight is zero, cannot estimate pose");
    exit(0);
  }
  double tt = 0.0;
  double tt2 = 0.0;
  for (const auto& particle : particles) {
    tt += pow(particle.pose.z() - pose.z(), 2);
    tt2 += pow(util.normalizeAngle(particle.pose.z() + M_PI) - normalized_angle, 2);
  }
  if (tt > tt2) {
    tt = tt2;
    pose.z() = util.normalizeAngle(normalized_angle - M_PI);
  }

  return pose;
}

pair<int, int> Mcl2d::mapToPixel(const float& x, const float& y) {
  int pixel_x = static_cast<int>((x - map_origin.x()) / image_resolution);
  int pixel_y = static_cast<int>((y - map_origin.y()) / image_resolution);
  return std::make_pair(pixel_x, pixel_y);
}

pair<float, float> Mcl2d::pixelToMap(const int& x, const int& y) {
  float map_x = (x * image_resolution) + map_origin.x();
  float map_y = (y * image_resolution) + map_origin.y();
  return std::make_pair(map_x, map_y);
}

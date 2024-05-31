#include "mcl_2d/mcl_2d.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <random>

default_random_engine gen;

void Mcl2d::loadMap(const string& yaml_path) {
  filesystem::path yaml_file_path = yaml_path;
  YAML::Node config = YAML::LoadFile(yaml_file_path.string());
  filesystem::path image_path = yaml_file_path.parent_path() / config["image"].as<string>();
  image_resolution = config["resolution"].as<double>();
  vector<double> origin = config["origin"].as<vector<double>>();
  map_origin << origin[0], origin[1], origin[2];
  // int negate = config["negate"].as<int>();
  // double occupied_thresh = config["occupied_thresh"].as<double>();
  // double free_thresh = config["free_thresh"].as<double>();

  cv::Mat src_map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (src_map_image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Failed to load map image: %s", image_path.c_str());
    return;
  }

  cv::Mat flipped_map_image;
  cv::flip(src_map_image, flipped_map_image, 0);
  // imshow("Map", flipped_map_image);
  // cv::waitKey(0);

  map_image = flipped_map_image.clone();
  displayPointOnMap(1.f, 2.f);

  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Map loaded successfully: %s", image_path.c_str());
}

void Mcl2d::generate_particles(const Vector3f& pose, const int particles_num) {
  particles.clear();

  // normal_distribution<float> x_pos(initial_pose[0], map_image.cols * image_resolution / 4.0);
  // normal_distribution<float> y_pos(initial_pose[1], map_image.rows * image_resolution / 4.0);
  // normal_distribution<float> theta_pos(initial_pose[2], M_PI / 4);

  std::normal_distribution<float> x_pos(pose[0], 0.1);
  std::normal_distribution<float> y_pos(pose[1], 0.1);
  std::normal_distribution<float> theta_pos(pose[2], M_PI / 8);

  for (int i = 0; i < particles_num; i++) {
    Particle particle_temp;

    particle_temp.pose = Matrix4f::Identity();
    particle_temp.pose(0, 3) = x_pos(gen);
    particle_temp.pose(1, 3) = y_pos(gen);
    particle_temp.pose(2, 3) = 0;
    particle_temp.pose.block<3, 3>(0, 0) = AngleAxisf(theta_pos(gen), Vector3f::UnitZ()).toRotationMatrix();

    particle_temp.score = 1 / (double)particles_num;
    particles.push_back(particle_temp);

    // RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Particle %d: x=%f, y=%f, yaw=%f, score=%f",
    //             i, particle_temp.pose(0, 3), particle_temp.pose(1, 3),
    //             std::atan2(particle_temp.pose(1, 0), particle_temp.pose(0, 0)), particle_temp.score);
  }
}

Vector3f Mcl2d::updateData(const Vector3f& pose, const vector<LaserPoint>& src_points, const int particles_num) {
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Updating data...");
  generate_particles(pose, particles_num);
  likelihood(src_points);
  return estimate_current_pose();
}

void Mcl2d::likelihood(const vector<LaserPoint>& src_points) {
  float max_score = 0;
  float score_sum = 0;

  for (auto& particle : particles) {
    float weight = 0;

    for (auto& point : src_points) {
      Eigen::Vector4f laser_point(point.x, point.y, 0, 1);
      Eigen::Vector4f map_point = particle.pose * laser_point;

      auto [image_pt_x, image_pt_y] = mapTopixel(map_point.x(), map_point.y());

      if (image_pt_x < 0 || image_pt_x >= map_image.cols || image_pt_y < 0 || image_pt_y >= map_image.rows)
        continue;
      else {
        double pixel_val = map_image.at<uchar>(image_pt_y, image_pt_x) / (double)255;
        weight += pixel_val;
      }
    }
    particle.score = particle.score + (weight / src_points.size());
    score_sum += particle.score;
    if (max_score < particle.score) {
      highest_weight_particle = particle;
      // highest_weight_particle.scan = laser;
      max_score = particle.score;
    }
  }
  for (int i = 0; i < (int)particles.size(); i++) {
    particles.at(i).score = particles.at(i).score / score_sum;  // 正規化
  }
}

Vector3f Mcl2d::estimate_current_pose() {
  Vector3f pose = Vector3f::Zero();

  if (highest_weight_particle.score > 0) {
    pose[0] = static_cast<float>(highest_weight_particle.pose(0, 3));
    pose[1] = static_cast<float>(highest_weight_particle.pose(1, 3));
    pose[2] = static_cast<float>(atan2f(highest_weight_particle.pose(1, 0), highest_weight_particle.pose(0, 0)));
    RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Highest Weight Particle  x: %f, y: %f, yaw: %f, score: %f", pose[0], pose[1], pose[2], highest_weight_particle.score);

  } else {
    RCLCPP_WARN(rclcpp::get_logger("mcl_2d"), "No particle with positive score");
  }
  return pose;
}

pair<int, int> Mcl2d::mapTopixel(const float& x, const float& y) {
  int pixel_x = static_cast<int>((x - map_origin[0]) / image_resolution);
  int pixel_y = static_cast<int>((y - map_origin[1]) / image_resolution);
  return std::make_pair(pixel_x, pixel_y);
}

pair<float, float> Mcl2d::pixelToMap(const int& x, const int& y) {
  float map_x = (x * image_resolution) + map_origin[0];
  float map_y = (y * image_resolution) + map_origin[1];
  return std::make_pair(map_x, map_y);
}

void Mcl2d::displayPointOnMap(const float& x, const float& y) {
  auto [pixel_x, pixel_y] = mapTopixel(x, y);
  auto [map_x, map_y] = pixelToMap(pixel_x, pixel_y);

  cv::Mat color_map_image;
  cv::cvtColor(map_image, color_map_image, cv::COLOR_GRAY2BGR);

  std::string text = "(" + std::to_string(map_x) + ", " + std::to_string(map_y) + ")";
  cv::putText(color_map_image, text, cv::Point(pixel_x, pixel_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
  cv::circle(color_map_image, cv::Point(pixel_x, pixel_y), 4, cv::Scalar(0, 255, 0), -1);

  auto [o_x, o_y] = mapTopixel(0.f, 0.f);
  cv::circle(color_map_image, cv::Point(o_x, o_y), 2, cv::Scalar(0, 0, 255), -1);

  cv::imshow("Map Image", color_map_image);
  cv::waitKey(0);  // キー入力を待つ
  exit(0);
}
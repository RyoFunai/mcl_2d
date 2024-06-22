#include "mcl_2d/mcl_2d.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
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

  map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (map_image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Failed to load map image: %s", image_path.c_str());
    return;
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
  auto start = std::chrono::high_resolution_clock::now();  // 計測開始

  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Updating data...");

  particles_temp.clear();
  resampled_particles.clear();

  particles_temp = generate_particles(pose, particles_num);
  likelihood(src_points, particles_temp);
  if (normalizeBelief(particles_temp) > 0.000001) {
    resampled_particles = systematic_resample(particles_temp);
  } else {
    // resetWeight();
    RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "No particle with positive score");
  }
  Vector3f current_pose = estimate_current_pose(particles_temp);

  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "updateData function took %lld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
  return current_pose;
}

vector<Particle> Mcl2d::generate_particles(const Vector3f& pose, const int particles_num) {
  vector<Particle> particles;

  std::normal_distribution<float> x_pos(pose.x(), 0.2);
  std::normal_distribution<float> y_pos(pose.y(), 0.2);
  std::normal_distribution<float> theta_pos(pose.z(), M_PI / 8);

  for (int i = 0; i < particles_num; i++) {
    Particle particle_temp;

    particle_temp.pose = Eigen::Matrix3f::Identity();
    particle_temp.pose(0, 2) = x_pos(gen);
    particle_temp.pose(1, 2) = y_pos(gen);
    particle_temp.pose.block<2, 2>(0, 0) = Eigen::Rotation2Df(theta_pos(gen)).toRotationMatrix();

    particle_temp.score = 1 / (double)particles_num;
    particles.push_back(particle_temp);
  }
  return particles;
}

void Mcl2d::likelihood(const vector<LaserPoint>& src_points, vector<Particle>& particles) {
  float max_score = 0;
  for (auto& particle : particles) {
    float weight = 0;

    for (auto& point : src_points) {
      Eigen::Vector3f laser_point(point.x, point.y, 1);
      Eigen::Vector3f map_point = particle.pose * laser_point;
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

vector<Particle> Mcl2d::systematic_resample(vector<Particle>& particles) {
  float step = 1 / particles.size();

  std::uniform_real_distribution<float> dist(0.0, step);
  float random_start_point = dist(gen);

  float cumulative_sum = particles[0].score;
  int count = 0;
  std::vector<Particle> new_particles;
  for (int i = 0; i < particles.size(); ++i) {
    float sampling_point = random_start_point + i * step;
    while (sampling_point > cumulative_sum) {
      count++;
      cumulative_sum += particles[count].score;
    }
    new_particles.push_back(particles[count]);
  }
}

double Mcl2d::normalizeBelief(vector<Particle>& particles) {
  double sum = 0.0;
  for (const auto& p : particles) {
    sum += p.score;
  }

  if (sum < 0.000000000001) {
    return sum;
  }

  for (auto& p : particles) {
    p.score /= sum;
  }

  return sum;
}

Vector3f Mcl2d::estimate_current_pose(const vector<Particle>& particles) {
  Vector3f pose = Vector3f::Zero();

  if (highest_weight_particle.score > 0) {
    float x_all = 0;
    float y_all = 0;
    float r11 = 0;
    float r21 = 0;
    for (int i = 0; i < (int)particles.size(); i++) {
      float const score = particles.at(i).score;
      x_all = x_all + particles.at(i).pose(0, 2) * score;
      y_all = y_all + particles.at(i).pose(1, 2) * score;
      r11 = r11 + particles.at(i).pose(0, 0) * score;
      r21 = r21 + particles.at(i).pose(1, 0) * score;
    }
    pose.x() = (double)x_all;
    pose.y() = (double)y_all;
    pose.z() = (double)atan2f(r21, r11);
    // RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Highest Weight Particle  x: %f, y: %f, yaw: %f, score: %f", pose[0], pose[1], pose[2], highest_weight_particle.score);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("mcl_2d"), "No particle with positive score");
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

void Mcl2d::displayPointOnMap(const float& x, const float& y) {
  auto [pixel_x, pixel_y] = mapToPixel(x, y);
  auto [map_x, map_y] = pixelToMap(pixel_x, pixel_y);

  cv::Mat color_map_image;
  cv::cvtColor(map_image, color_map_image, cv::COLOR_GRAY2BGR);

  std::string text = "(" + std::to_string(map_x) + ", " + std::to_string(map_y) + ")";
  cv::putText(color_map_image, text, cv::Point(pixel_x, pixel_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
  cv::circle(color_map_image, cv::Point(pixel_x, pixel_y), 4, cv::Scalar(0, 255, 0), -1);

  auto [o_x, o_y] = mapToPixel(0.f, 0.f);
  cv::circle(color_map_image, cv::Point(o_x, o_y), 2, cv::Scalar(0, 0, 255), -1);

  cv::imshow("Map Image", color_map_image);
  cv::waitKey(0);  // キー入力を待つ
  exit(0);
}

std::vector<LaserPoint> Mcl2d::getOverlappingPoints(const Vector3f& position, const std::vector<LaserPoint>& points) {
  std::vector<LaserPoint> overlapping_points;

  // 位置をピクセル座標に変換
  // auto [pixel_x, pixel_y] = mapToPixel(position[0], position[1]);

  // 位置を同次座標に変換
  // Eigen::Vector4f position_homogeneous(position[0], position[1], position[2], 1.0);

  for (const auto& point : points) {
    // 点群の各点をピクセル座標に変換
    // Eigen::Vector4f laser_point(point.x, point.y, 0, 1);             // 4次元に変更
    // Eigen::Vector4f map_point = position_homogeneous + laser_point;  // 同次座標での変換
    auto [image_pt_x, image_pt_y] = mapToPixel(point.x, point.y);

    // 画像の範囲内かチェック
    if (image_pt_x >= 0 && image_pt_x < map_image.cols && image_pt_y >= 0 && image_pt_y < map_image.rows) {
      // 障害物と被っているかチェック
      if (map_image.at<uchar>(image_pt_y, image_pt_x) == 0) {  // 0=障害物
        overlapping_points.push_back(point);
      }
    }
  }

  return overlapping_points;
}

void Mcl2d::onMouse(int event, int x, int y, int, void* userdata) {
  if (event == cv::EVENT_MOUSEMOVE) {
    cv::Mat* image = reinterpret_cast<cv::Mat*>(userdata);
    if (x >= 0 && x < image->cols && y >= 0 && y < image->rows) {
      uchar pixel = image->at<uchar>(y, x);
      RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Pixel value at (%d, %d): %d", x, y, static_cast<int>(pixel));
    }
  }
}

void Mcl2d::displayLaserPoints(const std::vector<LaserPoint>& points) {
  // 画像がグレースケールの場合、カラーに変換
  cv::Mat color_map_image;
  if (map_image.channels() == 1) {
    cv::cvtColor(map_image, color_map_image, cv::COLOR_GRAY2BGR);
  } else {
    color_map_image = map_image.clone();
  }

  // 点群を画像上に描画
  for (const auto& point : points) {
    auto [pixel_x, pixel_y] = mapToPixel(point.x, point.y);
    if (pixel_x >= 0 && pixel_x < color_map_image.cols && pixel_y >= 0 && pixel_y < color_map_image.rows) {
      cv::circle(color_map_image, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(0, 0, 255), -1);  // 赤色の点を描画
    }
  }

  cv::imshow("Laser Points", color_map_image);
  cv::waitKey(0);  // キー入力を待つ
}
#include "mcl_2d/mcl_2d.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <random>

default_random_engine gen;

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
    particles = generate_particles_normal(pose, particles_num);
    is_first_time = false;
  } else {
    sampling(diff_pose, particles);
  }

  // particles = generate_particles_normal(pose, particles_num);

  likelihood(src_points, particles);
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "updateData function took %lld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());

  if (normalizeBelief(particles) > 1.0E-6) {
    simple_resample(particles);
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

vector<Particle> Mcl2d::generate_particles_normal(const Vector3f& pose, const int particles_num) {
  vector<Particle> particles;

  std::normal_distribution<float> x_pos(pose.x(), 0.1);
  std::normal_distribution<float> y_pos(pose.y(), 0.1);
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

// void Mcl2d::sampling(Vector3f& diffPose, vector<Particle>& particles) {
//   double delta_length = sqrt(pow(diffPose.x(), 2) + pow(diffPose.y(), 2));
//   double delta_rot1 = atan2(diffPose.y(), diffPose.x());  //始点角度差分
//   double delta_rot2 = diffPose.z() - delta_rot1;  //終点角度差分

//   RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "diffPose: %f, %f, %f", diffPose.x(), diffPose.y(), diffPose.z() * 180 / M_PI);
//   RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "delta_length: %f, delta_rot1: %f, delta_rot2: %f", delta_length, delta_rot1 * 180 / M_PI, delta_rot2 * 180 / M_PI);

//   std::default_random_engine generator;
//   if (delta_rot1 > M_PI)
//     delta_rot1 -= (2 * M_PI);
//   if (delta_rot1 < -M_PI)
//     delta_rot1 += (2 * M_PI);
//   if (delta_rot2 > M_PI)
//     delta_rot2 -= (2 * M_PI);
//   if (delta_rot2 < -M_PI)
//     delta_rot2 += (2 * M_PI);
//   //// Add noises to trans/rot1/rot2
//   double trans_noise_coeff = odom_convariance_[2] * fabs(delta_length) + odom_convariance_[3] * fabs(delta_rot1 + delta_rot2);
//   double rot1_noise_coeff = odom_convariance_[0] * fabs(delta_rot1) + odom_convariance_[1] * fabs(delta_length);
//   double rot2_noise_coeff = odom_convariance_[0] * fabs(delta_rot2) + odom_convariance_[1] * fabs(delta_length);

//   float scoreSum = 0;
//   for (int i = 0; i < particles.size(); i++) {
//     std::normal_distribution<double> gaussian_distribution(0, 1);

//     delta_length = delta_length + gaussian_distribution(gen) * trans_noise_coeff;
//     delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff;
//     delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff;

//     double x = delta_length * cos(delta_rot1) + gaussian_distribution(gen) * odom_convariance_[4];
//     double y = delta_length * sin(delta_rot1) + gaussian_distribution(gen) * odom_convariance_[5];
//     double theta = delta_rot1 + delta_rot2 + gaussian_distribution(gen) * odom_convariance_[0] * (M_PI / 180.0);

//     Eigen::Matrix3f diff_odom_w_noise = Eigen::Matrix3f::Identity();
//     diff_odom_w_noise(0, 2) = x;
//     diff_odom_w_noise(1, 2) = y;
//     diff_odom_w_noise.block<2, 2>(0, 0) = Eigen::Rotation2Df(theta).toRotationMatrix();

//     particles.at(i).pose = particles.at(i).pose * diff_odom_w_noise;
//   }
// }
void Mcl2d::sampling(Vector3f& diffPose, vector<Particle>& particles) {
  // 移動量を表す変換行列を作成
  Eigen::Matrix3f diff_transform = Eigen::Matrix3f::Identity();
  diff_transform(0, 2) = diffPose.x();
  diff_transform(1, 2) = diffPose.y();
  diff_transform.block<2, 2>(0, 0) = Eigen::Rotation2Df(diffPose.z()).toRotationMatrix();

  // 各パーティクルに移動量を適用
  for (auto& particle : particles) {
    particle.pose = particle.pose * diff_transform;

    // パーティクルの角度も正規化
    double current_angle = atan2(particle.pose(1, 0), particle.pose(0, 0));
    double normalized_angle = util.normalizeAngle(current_angle);
    particle.pose.block<2, 2>(0, 0) = Eigen::Rotation2Df(normalized_angle).toRotationMatrix();
  }
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

void Mcl2d::simple_resample(vector<Particle>& particles) {
  vector<Particle> new_particles;
  new_particles.reserve(particles.size());

  // 累積重みの計算
  vector<double> cumulative_weights(particles.size());
  cumulative_weights[0] = particles[0].score;
  for (size_t i = 1; i < particles.size(); ++i) {
    cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].score;
  }

  // リサンプリング
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  for (size_t i = 0; i < particles.size(); ++i) {
    double random_weight = dist(gen) * cumulative_weights.back();
    auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_weight);
    size_t index = std::distance(cumulative_weights.begin(), it);
    new_particles.push_back(particles[index]);
    new_particles.back().score = 1.0 / particles.size();  // 重みをリセット
  }

  particles = std::move(new_particles);
}

void Mcl2d::systematic_resample(vector<Particle>& particles) {
  float step = 1.0f / particles.size();

  std::uniform_real_distribution<float> dist(0.0, step);
  float random_start_point = dist(gen);

  float cumulative_sum = particles[0].score;
  int count = 0;
  vector<Particle> new_particles;
  new_particles.reserve(particles.size());  // 事前にメモリを確保

  for (int i = 0; i < particles.size(); ++i) {
    float sampling_point = random_start_point + i * step;
    while (sampling_point > cumulative_sum) {
      count++;
      cumulative_sum += particles[count].score;
    }
    new_particles.push_back(particles[count]);
  }

  particles = std::move(new_particles);  // 新しいパーティクルを元のベクトルに反映
}

double Mcl2d::normalizeBelief(vector<Particle>& particles) {
  double sum = 0.0;
  for (const auto& p : particles) {
    sum += p.score;
  }

  if (sum < 1.0E-12) {
    return sum;
  }

  for (auto& p : particles) {
    p.score /= sum;
  }

  return sum;
}

// double Mcl2d::normalizeBelief(vector<Particle>& particles) {
//   double sum = 0.0;
//   double min_score = std::numeric_limits<double>::max();
//   double max_score = std::numeric_limits<double>::lowest();

//   // 最小値と最大値を見つける
//   for (const auto& p : particles) {
//     sum += p.score;
//     min_score = std::min(min_score, static_cast<double>(p.score));
//     max_score = std::max(max_score, static_cast<double>(p.score));
//   }

//   if (sum < 1.0E-12 || max_score == min_score) {
//     // すべての重みが同じ場合や、合計が非常に小さい場合は均等に分配
//     double uniform_weight = 1.0 / particles.size();
//     for (auto& p : particles) {
//       p.score = uniform_weight;
//     }
//     return sum;
//   }

//   // 正規化と0-1スケーリングを同時に行う
//   for (auto& p : particles) {
//     p.score = (p.score - min_score) / (max_score - min_score);
//   }

//   return sum;
// }

// Vector3f Mcl2d::estimate_current_pose(const vector<Particle>& particles) {
//   Vector3f pose = Vector3f::Zero();
//   double total_weight = 0.0;
//   double cos_sum = 0.0;
//   double sin_sum = 0.0;

//   for (const auto& particle : particles) {
//     double weight = particle.score;
//     total_weight += weight;

//     // 位置の重み付き和を計算
//     pose.x() += particle.pose(0, 2) * weight;
//     pose.y() += particle.pose(1, 2) * weight;

//     // 角度の重み付きsin和とcos和を計算
//     double angle = atan2(particle.pose(1, 0), particle.pose(0, 0));
//     cos_sum += cos(angle) * weight;
//     sin_sum += sin(angle) * weight;
//   }

//   if (total_weight > 0) {
//     // 重み付き平均を計算
//     pose.x() /= total_weight;
//     pose.y() /= total_weight;
//     pose.z() = atan2(sin_sum, cos_sum);
//   } else {
//     RCLCPP_WARN(rclcpp::get_logger("mcl_2d"), "Total weight is zero, cannot estimate pose");
//   }

//   RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "est pose : %f, %f, %f", pose.x(), pose.y(), pose.z());
//   return pose;
// }

Vector3f Mcl2d::estimate_current_pose_V3(const vector<ParticleV3>& particles) {
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

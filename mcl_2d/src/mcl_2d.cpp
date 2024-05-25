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
  origin_ << origin[0], origin[1], origin[2];
  // int negate = config["negate"].as<int>();
  // double occupied_thresh = config["occupied_thresh"].as<double>();
  // double free_thresh = config["free_thresh"].as<double>();

  cv::Mat map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (map_image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Failed to load map image: %s", image_path.c_str());
    return;
  }

  cv::Mat flipped_map_image;
  cv::flip(map_image, flipped_map_image, 0);
  // imshow("Map", flipped_map_image);
  // cv::waitKey(0);

  gridMap = flipped_map_image.clone();
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Map loaded successfully: %s", image_path.c_str());
}

void Mcl2d::init_particles(const Vector3d& initial_pose, const int particles_num) {
  particles.clear();

  // normal_distribution<float> x_pos(initial_pose[0], gridMap.cols * image_resolution / 4.0);
  // normal_distribution<float> y_pos(initial_pose[1], gridMap.rows * image_resolution / 4.0);
  // normal_distribution<float> theta_pos(initial_pose[2], M_PI / 4);

  Vector3d map_initial_pose = initial_pose - origin_;
  std::normal_distribution<float> x_pos(initial_pose[0], 0.1);  // Standard deviation of 1.0
  std::normal_distribution<float> y_pos(initial_pose[1], 0.1);  // Standard deviation of 1.0
  std::normal_distribution<float> theta_pos(initial_pose[2], M_PI / 8);

  for (int i = 0; i < particles_num; i++) {
    Particle particle_temp;

    particle_temp.pose = Matrix4f::Identity();
    particle_temp.pose(0, 3) = x_pos(gen);
    particle_temp.pose(1, 3) = y_pos(gen);
    particle_temp.pose(2, 3) = 0;
    particle_temp.pose.block<3, 3>(0, 0) = AngleAxisf(theta_pos(gen), Vector3f::UnitZ()).toRotationMatrix();

    particle_temp.score = 1 / (double)particles_num;
    particles.push_back(particle_temp);

    RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Particle %d: x=%f, y=%f, yaw=%f, score=%f",
                i, particle_temp.pose(0, 3), particle_temp.pose(1, 3),
                std::atan2(particle_temp.pose(1, 0), particle_temp.pose(0, 0)), particle_temp.score);
  }
}
// int Mcl2d::setup(const int particles_num, const float odomCovariance[6], const Matrix4f tf_laser2robot, const Matrix4f initial_pose) {
//   //--YOU CAN CHANGE THIS PARAMETERS BY YOURSELF--//
//   this->particles_num = particles_num;  // Number of Particles.
//   minOdomDistance = 0.001;              // [m]
//   minOdomAngle = 0.1;                   // [deg]
//   repropagateCountNeeded = 1;           // [num]

//   for (int i = 0; i < 6; i++) {
//     this->odomCovariance[i] = odomCovariance[i];
//   }

//   //--DO NOT TOUCH THIS PARAMETERS--//
//   image_resolution = 0.05;  // [m] per [pixel]

//   this->tf_laser2robot = tf_laser2robot;
//   this->initial_pose = initial_pose;

//   VectorXf initial_xyzrpy = tool::eigen2xyzrpy(initial_pose);
//   x = initial_xyzrpy[0];
//   y = initial_xyzrpy[1];
//   angle = initial_xyzrpy[5];

//   mapCenterX = 0;             // [m]
//   mapCenterY = 0;             // [m]
//   isOdomInitialized = false;  // Will be true when first data incoming.
//   predictionCounter = 0;

//   initializeParticles();  // Initialize particles.
//   showInMap();

//   return 0;
// }
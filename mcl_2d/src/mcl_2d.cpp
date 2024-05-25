#include "mcl_2d/mcl_2d.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>


void Mcl2d::loadMap(const std::string& yaml_path) {
  std::filesystem::path yaml_file_path = yaml_path;
  YAML::Node config = YAML::LoadFile(yaml_file_path.string());
  std::filesystem::path image_path = yaml_file_path.parent_path() / config["image"].as<std::string>();
  double resolution = config["resolution"].as<double>();
  std::vector<double> origin = config["origin"].as<std::vector<double>>();
  int negate = config["negate"].as<int>();
  double occupied_thresh = config["occupied_thresh"].as<double>();
  double free_thresh = config["free_thresh"].as<double>();

  cv::Mat map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (map_image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mcl_2d"), "Failed to load map image: %s", image_path.c_str());
    return;
  }

  cv::Mat flipped_map_image;
  cv::flip(map_image, flipped_map_image, 0);
  imshow("Map", flipped_map_image);
  // cv::waitKey(0);
  RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "Map loaded successfully: %s", image_path.c_str());
}

// void mcl::initializeParticles() {
//   particles.clear();

//   // Extract initial x, y, and yaw from initial_pose
//   float initial_x;
//   float initial_y;
//   float initial_yaw;

//   // Define distributions around the initial pose
//   std::uniform_real_distribution<float> x_pos(initial_x - gridMapCV.cols * imageResolution / 4.0,
//                                               initial_x + gridMapCV.cols * imageResolution / 4.0);
//   std::uniform_real_distribution<float> y_pos(initial_y - gridMapCV.rows * imageResolution / 4.0,
//                                               initial_y + gridMapCV.rows * imageResolution / 4.0);
//   std::uniform_real_distribution<float> theta_pos(initial_yaw - M_PI / 4, initial_yaw + M_PI / 4);

//   // Set particles by random distribution
//   for (int i = 0; i < numOfParticle; i++) {
//     Particle particle_temp;

//     particle_temp.pose = initial_pose;
//     particle_temp.pose(0, 3) = x_pos(gen);
//     particle_temp.pose(1, 3) = y_pos(gen);
//     particle_temp.pose(2, 3) = 0;
//     particle_temp.pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(theta_pos(gen), Eigen::Vector3f::UnitZ()).toRotationMatrix();

//     particle_temp.score = 1 / (double)numOfParticle;
//     particles.push_back(particle_temp);
//   }
// }
// int mcl::setup(const int numOfParticle, const float odomCovariance[6], const Eigen::Matrix4f tf_laser2robot, const Eigen::Matrix4f initial_pose) {
//   //--YOU CAN CHANGE THIS PARAMETERS BY YOURSELF--//
//   this->numOfParticle = numOfParticle;  // Number of Particles.
//   minOdomDistance = 0.001;              // [m]
//   minOdomAngle = 0.1;                   // [deg]
//   repropagateCountNeeded = 1;           // [num]

//   for (int i = 0; i < 6; i++) {
//     this->odomCovariance[i] = odomCovariance[i];
//   }

//   //--DO NOT TOUCH THIS PARAMETERS--//
//   imageResolution = 0.05;  // [m] per [pixel]

//   this->tf_laser2robot = tf_laser2robot;
//   this->initial_pose = initial_pose;

//   Eigen::VectorXf initial_xyzrpy = tool::eigen2xyzrpy(initial_pose);
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
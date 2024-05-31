#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "scan.hpp"

using namespace Eigen;
using namespace std;

class Mcl2d {
 public:
  struct Particle {
    Eigen::Matrix4f pose;
    float score;
  };

  void loadMap(const string& yaml_path);
  void generate_particles(const Vector3f& initial_pose, const int particles_num);
  vector<Particle> getParticles() { return particles; };
  Vector3f updateData(const Vector3f& pose, const vector<LaserPoint>& src_points, const int particles_num);

 private:
  void likelihood(const vector<LaserPoint>& src_points);
  Vector3f estimate_current_pose();
  pair<int, int> mapTopixel(const float& x, const float& y);
  pair<float, float> pixelToMap(const int& x, const int& y);
  void displayPointOnMap(const float& x, const float& y);

  cv::Mat map_image;
  rclcpp::Node::SharedPtr node_;
  vector<Particle> particles;
  double image_resolution;
  Vector3f map_origin;
  Particle highest_weight_particle;
};

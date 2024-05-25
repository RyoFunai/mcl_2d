#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace Eigen;
using namespace std;

class Mcl2d
{
public:
  struct Particle{
    Eigen::Matrix4f pose;
    float score;
  };

  void loadMap(const string& yaml_path);
  void init_particles(const Vector3d& initial_pose, const int particles_num);
  vector<Particle> getParticles() { return particles; };




private:
  cv::Mat gridMap;
  rclcpp::Node::SharedPtr node_;
  vector<Particle> particles;
  double image_resolution;
  Vector3d origin_;
};

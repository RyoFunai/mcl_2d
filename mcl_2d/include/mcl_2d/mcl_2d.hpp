#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "mcl_2d/particle_sampler.hpp"
#include "mcl_2d/util.hpp"
#include "mcl_2d/visibility_control.hpp"
#include "scan.hpp"

using namespace Eigen;
using namespace std;

class MCL2D_CORE_EXPORT Mcl2d {
 public:
  void setup(const string& yaml_path, const vector<double>& odom_convariance);
  vector<Particle> getParticles() { return particles; };
  // vector<Particle> getResampledParticles() { return resampled_particles; };
  Vector3f updateData(const Vector3f& pose, const vector<LaserPoint>& src_points, const int particles_num);
  Vector3f estimate_current_pose(const vector<Particle>& particles);

 private:
  void loadMap(const string& yaml_path);
  void likelihood(const vector<LaserPoint>& src_points, vector<Particle>& particles);
  double generateGaussianNoise(double mean, double stddev);

  pair<int, int> mapToPixel(const float& x, const float& y);
  pair<float, float> pixelToMap(const int& x, const int& y);

  Util util;
  std::unique_ptr<ParticleSampler> particle_sampler;

  cv::Mat map_image;
  rclcpp::Node::SharedPtr node_;
  double image_resolution;
  Vector3f map_origin;
  Vector3f pre_pose = Vector3f::Zero();
  Vector3f diff_pose = Vector3f::Zero();
  Particle highest_weight_particle;
  bool is_first_time{true};
  vector<double> odom_convariance_;
  vector<Particle> particles;
};

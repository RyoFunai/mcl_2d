#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

class Mcl2d
{
public:
  void loadMap(const std::string& yaml_path);
  struct Particle{
    Eigen::Matrix4f pose;
    float score;
    Eigen::Matrix4Xf scan; // Only for maximum probability particle.
  };

private:
  cv::Mat gridMap;
  rclcpp::Node::SharedPtr node_;

};

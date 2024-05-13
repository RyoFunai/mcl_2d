#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <cmath>

class Mcl2d
{
public:
  struct Particle{
    Eigen::Matrix4f pose;
    float score;
    Eigen::Matrix4Xf scan; // Only for maximum probability particle.
  };

private:
};

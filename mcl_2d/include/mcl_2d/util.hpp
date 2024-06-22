#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "scan.hpp"

using namespace std;
using namespace Eigen;

class Util {
 public:
  double quaternionToYaw(const double& w, const double& x, const double& y, const double& z);
  vector<LaserPoint> transformToMapCoordinates(const std::vector<LaserPoint>& src_points, const Vector3f& pose);

 private:
};
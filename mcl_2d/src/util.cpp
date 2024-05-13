#include "mcl_2d/util.hpp"
#include <cmath>

double Util::quaternionToYaw(const double& w, const double& x, const double& y, const double& z) {
    double roll, pitch, yaw;
    // asinの範囲を超えないようにクランプ
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}


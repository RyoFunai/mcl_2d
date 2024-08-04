#pragma once

#include <vector>
#include <random>
#include <Eigen/Dense>

struct Particle {
  Eigen::Vector3f pose;
  float score;
};


class ParticleSampler {
public:
    ParticleSampler();

    std::vector<Particle> generate_particles_normal(const Eigen::Vector3f& pose, int particles_num);
    void sampling(Eigen::Vector3f& diffPose, std::vector<Particle>& particles);
    void simple_resample(std::vector<Particle>& particles);
    void systematic_resample(std::vector<Particle>& particles);
    double normalizeBelief(std::vector<Particle>& particles);

private:
    std::array<double, 6> odom_convariance_;

    float normalizeAngle(float angle);
};
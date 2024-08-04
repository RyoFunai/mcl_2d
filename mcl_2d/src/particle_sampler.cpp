#include "mcl_2d/particle_sampler.hpp"

#include <cmath>

std::default_random_engine gen;

ParticleSampler::ParticleSampler() {
  // オドメトリの共分散行列を初期化
  odom_convariance_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
}

std::vector<Particle> ParticleSampler::generate_particles_normal(const Eigen::Vector3f& pose, int particles_num) {
  std::vector<Particle> particles;

  std::normal_distribution<float> x_pos(pose.x(), 0.1);
  std::normal_distribution<float> y_pos(pose.y(), 0.1);
  std::normal_distribution<float> theta_pos(pose.z(), M_PI / 8);

  for (int i = 0; i < particles_num; i++) {
    Particle particle_temp;

    particle_temp.pose.x() = x_pos(gen);
    particle_temp.pose.y() = y_pos(gen);
    particle_temp.pose.z() = theta_pos(gen);

    particle_temp.score = 1.0 / particles_num;
    particles.push_back(particle_temp);
  }
  return particles;
}

void ParticleSampler::sampling(Eigen::Vector3f& diffPose, std::vector<Particle>& particles) {
  for (auto& particle : particles) {
    particle.pose.x() += diffPose.x() * cos(particle.pose.z()) - diffPose.y() * sin(particle.pose.z());
    particle.pose.y() += diffPose.x() * sin(particle.pose.z()) + diffPose.y() * cos(particle.pose.z());
    particle.pose.z() += diffPose.z();
    particle.pose.z() = normalizeAngle(particle.pose.z());
  }
}

// void Mcl2d::sampling(Vector3f& diffPose, vector<Particle>& particles) {
//   double delta_length = sqrt(pow(diffPose.x(), 2) + pow(diffPose.y(), 2));
//   double delta_rot1 = atan2(diffPose.y(), diffPose.x());  //始点角度差分
//   double delta_rot2 = diffPose.z() - delta_rot1;  //終点角度差分

//   RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "diffPose: %f, %f, %f", diffPose.x(), diffPose.y(), diffPose.z() * 180 / M_PI);
//   RCLCPP_INFO(rclcpp::get_logger("mcl_2d"), "delta_length: %f, delta_rot1: %f, delta_rot2: %f", delta_length, delta_rot1 * 180 / M_PI, delta_rot2 * 180 / M_PI);

//   std::default_random_engine generator;
//   if (delta_rot1 > M_PI)
//     delta_rot1 -= (2 * M_PI);
//   if (delta_rot1 < -M_PI)
//     delta_rot1 += (2 * M_PI);
//   if (delta_rot2 > M_PI)
//     delta_rot2 -= (2 * M_PI);
//   if (delta_rot2 < -M_PI)
//     delta_rot2 += (2 * M_PI);
//   //// Add noises to trans/rot1/rot2
//   double trans_noise_coeff = odom_convariance_[2] * fabs(delta_length) + odom_convariance_[3] * fabs(delta_rot1 + delta_rot2);
//   double rot1_noise_coeff = odom_convariance_[0] * fabs(delta_rot1) + odom_convariance_[1] * fabs(delta_length);
//   double rot2_noise_coeff = odom_convariance_[0] * fabs(delta_rot2) + odom_convariance_[1] * fabs(delta_length);

//   float scoreSum = 0;
//   for (int i = 0; i < particles.size(); i++) {
//     std::normal_distribution<double> gaussian_distribution(0, 1);

//     delta_length = delta_length + gaussian_distribution(gen) * trans_noise_coeff;
//     delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff;
//     delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff;

//     double x = delta_length * cos(delta_rot1) + gaussian_distribution(gen) * odom_convariance_[4];
//     double y = delta_length * sin(delta_rot1) + gaussian_distribution(gen) * odom_convariance_[5];
//     double theta = delta_rot1 + delta_rot2 + gaussian_distribution(gen) * odom_convariance_[0] * (M_PI / 180.0);

//     Eigen::Matrix3f diff_odom_w_noise = Eigen::Matrix3f::Identity();
//     diff_odom_w_noise(0, 2) = x;
//     diff_odom_w_noise(1, 2) = y;
//     diff_odom_w_noise.block<2, 2>(0, 0) = Eigen::Rotation2Df(theta).toRotationMatrix();

//     particles.at(i).pose = particles.at(i).pose * diff_odom_w_noise;
//   }
// }

void ParticleSampler::simple_resample(std::vector<Particle>& particles) {
  std::vector<Particle> new_particles;
  new_particles.reserve(particles.size());

  std::vector<double> cumulative_weights(particles.size());
  cumulative_weights[0] = particles[0].score;
  for (size_t i = 1; i < particles.size(); ++i) {
    cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].score;
  }

  std::uniform_real_distribution<double> dist(0.0, 1.0);
  for (size_t i = 0; i < particles.size(); ++i) {
    double random_weight = dist(gen) * cumulative_weights.back();
    auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_weight);
    size_t index = std::distance(cumulative_weights.begin(), it);
    new_particles.push_back(particles[index]);
    new_particles.back().score = 1.0 / particles.size();
  }

  particles = std::move(new_particles);
}

void ParticleSampler::systematic_resample(std::vector<Particle>& particles) {
  float step = 1.0f / particles.size();

  std::uniform_real_distribution<float> dist(0.0, step);
  float random_start_point = dist(gen);

  float cumulative_sum = particles[0].score;
  int count = 0;
  std::vector<Particle> new_particles;
  new_particles.reserve(particles.size());

  for (size_t i = 0; i < particles.size(); ++i) {
    float sampling_point = random_start_point + i * step;
    while (sampling_point > cumulative_sum) {
      count++;
      cumulative_sum += particles[count].score;
    }
    new_particles.push_back(particles[count]);
  }

  particles = std::move(new_particles);
}

double ParticleSampler::normalizeBelief(std::vector<Particle>& particles) {
  double sum = 0.0;
  for (const auto& p : particles) {
    sum += p.score;
  }

  if (sum < 1.0E-12) {
    return sum;
  }

  for (auto& p : particles) {
    p.score /= sum;
  }

  return sum;
}

float ParticleSampler::normalizeAngle(float angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}
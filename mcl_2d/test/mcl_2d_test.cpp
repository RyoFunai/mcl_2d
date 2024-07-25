#include "mcl_2d/mcl_2d.hpp"

#include <gtest/gtest.h>
#include <iostream>

class TestPrinter : public testing::EmptyTestEventListener {
 public:
  virtual void OnTestStart(const testing::TestInfo& test_info) {
    std::cout << "Running test: " << test_info.test_case_name() << "." << test_info.name() << std::endl;
  }
};

class Mcl2dTest : public ::testing::Test {
 protected:
  Mcl2d mcl;

  void SetUp() override {
    // テスト前の共通セットアップ
  }

  void TearDown() override {
    // テスト後のクリーンアップ
  }

  Particle createParticle(double x, double y, double theta, float weight) {
    Eigen::Matrix3f pose = Eigen::Matrix3f::Identity();
    pose(0, 2) = static_cast<float>(x);
    pose(1, 2) = static_cast<float>(y);
    pose.block<2, 2>(0, 0) = Eigen::Rotation2Df(static_cast<float>(theta)).toRotationMatrix();
    return {pose, weight};
  }
};

TEST_F(Mcl2dTest, EstimateCurrentPoseSingleParticle) {
  vector<Particle> particles;
  particles.push_back(createParticle(1.0, 2.0, M_PI / 4, 1.0f));

  Vector3f result = mcl.estimate_current_pose(particles);

  EXPECT_NEAR(result.x(), 1.0, 1e-6);
  EXPECT_NEAR(result.y(), 2.0, 1e-6);
  EXPECT_NEAR(result.z(), M_PI / 4, 1e-6);
}

TEST_F(Mcl2dTest, EstimateCurrentPoseMultipleParticles) {
  vector<Particle> particles;
  particles.push_back(createParticle(1.0, 1.0, 0.0, 0.2f));
  particles.push_back(createParticle(2.0, 2.0, M_PI, 0.3f));
  particles.push_back(createParticle(3.0, 3.0, M_PI, 0.5f));

  Vector3f result = mcl.estimate_current_pose(particles);

  std::cout << "*********************************************" << std::endl;

  vector<ParticleV3> particles_v3;
  particles_v3.push_back({Vector3f(1.0, 1.0, 0.0), 0.2f});
  particles_v3.push_back({Vector3f(2.0, 2.0, M_PI / 2), 0.3f});
  particles_v3.push_back({Vector3f(3.0, 3.0, M_PI), 0.5f});

  Vector3f result_v3 = mcl.estimate_current_pose_V3(particles_v3);

  EXPECT_NEAR(result.x(), 2.0, 1e-6);
  EXPECT_NEAR(result.y(), 2.0, 1e-6);
  EXPECT_NEAR(result.z(), M_PI / 4, 1e-6);

  EXPECT_NEAR(result_v3.x(), 2.3, 1e-6);
  EXPECT_NEAR(result_v3.y(), 2.3, 1e-6);
  EXPECT_NEAR(result_v3.z(), 2.356194, 1e-6);  // Approximately 3π/4
}

TEST_F(Mcl2dTest, EstimateCurrentPoseEmptyParticles) {
  vector<Particle> particles;

  Vector3f result = mcl.estimate_current_pose(particles);

  EXPECT_NEAR(result.x(), 0.0, 1e-6);
  EXPECT_NEAR(result.y(), 0.0, 1e-6);
  EXPECT_NEAR(result.z(), 0.0, 1e-6);
}

TEST_F(Mcl2dTest, EstimateCurrentPoseAngleWrapping) {
  vector<Particle> particles;
  particles.push_back(createParticle(0.0, 0.0, 3 * M_PI / 4, 0.5));
  particles.push_back(createParticle(0.0, 0.0, -3 * M_PI / 4, 0.5));

  Vector3f result = mcl.estimate_current_pose(particles);

  EXPECT_NEAR(result.x(), 0.0, 1e-6);
  EXPECT_NEAR(result.y(), 0.0, 1e-6);
  EXPECT_NEAR(result.z(), M_PI, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  
  // デフォルトリスナーを削除
  testing::TestEventListeners& listeners = testing::UnitTest::GetInstance()->listeners();
  delete listeners.Release(listeners.default_result_printer());

  // カスタムリスナーを追加
  listeners.Append(new TestPrinter);
  
  const char* test_filter = std::getenv("MCL_TEST_FILTER");
  if (test_filter) {
    testing::GTEST_FLAG(filter) = test_filter;
  } else {
    // デフォルトで特定のテストを実行
    testing::GTEST_FLAG(filter) = "Mcl2dTest.EstimateCurrentPoseEmptyParticles";
  }
  
  return RUN_ALL_TESTS();
}

/* 実行方法
MCL_TEST_FILTER="テスト名" colcon test --packages-select mcl_2d --event-handlers console_direct+
MCL_TEST_FILTER="テスト名" ./build/mcl_2d/mcl_2d_test
テスト名：Mcl2dTest.EstimateCurrentPoseMultipleParticles
          * で全てのテストを実行
*/
// FK / 야코비안 / IK 검증
//   FK       export_ik_reference.py 가 저장한 Python FK 결과 (targets.csv) 와 1e-9 이내
//   야코비안 FK 수치 미분과 비교
//   IK       결과를 FK 로 다시 계산해 허용 오차 이내
#include "omy_motion_cpp/csv.hpp"
#include "omy_motion_cpp/kinematics.hpp"

#include <gtest/gtest.h>

using namespace omy_motion;

namespace
{

const std::string kDir = OMY_RESULTS_DIR;

Vector6d readVec(const CsvTable & t, size_t row, const std::string & prefix)
{
  Vector6d v;
  for (int k = 0; k < kDof; ++k) {
    v[k] = t.num(row, prefix + std::to_string(k));
  }
  return v;
}

class KinematicsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    kin_ = std::make_unique<Kinematics>(readFile(kDir + "/omy.urdf"));
    targets_ = std::make_unique<CsvTable>(readCsv(kDir + "/targets.csv"));
  }
  static std::unique_ptr<Kinematics> kin_;
  static std::unique_ptr<CsvTable> targets_;
};
std::unique_ptr<Kinematics> KinematicsTest::kin_;
std::unique_ptr<CsvTable> KinematicsTest::targets_;

}  // namespace

TEST_F(KinematicsTest, ChainMatchesUrdf)
{
  const std::vector<std::string> expected{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  EXPECT_EQ(kin_->jointNames(), expected);
  EXPECT_NEAR(kin_->upper()[2], 2.6179938779914944, 1e-12);
  EXPECT_NEAR(kin_->lower()[0], -2 * M_PI, 1e-12);
}

TEST_F(KinematicsTest, FkMatchesPython)
{
  double worst = 0.0;
  for (size_t r = 0; r < targets_->rows.size(); ++r) {
    const Eigen::Matrix4d m = kin_->fk(readVec(*targets_, r, "g")).matrix();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        worst = std::max(worst, std::abs(m(i, j) - targets_->num(r, "T" + std::to_string(i) + std::to_string(j))));
      }
    }
  }
  EXPECT_LT(worst, 1e-9);
}

TEST_F(KinematicsTest, JacobianMatchesFiniteDifference)
{
  const double h = 1e-7;
  for (size_t r = 0; r < 50; ++r) {
    const Vector6d q = readVec(*targets_, r, "g");
    const Matrix6d jac = kin_->jacobian(q);
    const Eigen::Isometry3d t0 = kin_->fk(q);
    for (int k = 0; k < kDof; ++k) {
      Vector6d qh = q;
      qh[k] += h;
      const Eigen::Isometry3d t1 = kin_->fk(qh);
      const Eigen::Vector3d dp = (t1.translation() - t0.translation()) / h;
      const Eigen::Vector3d dw = rotationError(t1.linear(), t0.linear()) / h;
      EXPECT_LT((jac.block<3, 1>(0, k) - dp).norm(), 1e-5) << "row " << r << " joint " << k;
      EXPECT_LT((jac.block<3, 1>(3, k) - dw).norm(), 1e-5) << "row " << r << " joint " << k;
    }
  }
}

TEST_F(KinematicsTest, IkFromNearStartConverges)
{
  int ok = 0;
  for (size_t r = 0; r < targets_->rows.size(); ++r) {
    Eigen::Isometry3d target = kin_->fk(readVec(*targets_, r, "g"));
    const IkResult res = kin_->ik(target, readVec(*targets_, r, "near"));
    if (res.success) {
      ++ok;
      const auto [pos, rot] = kin_->poseError(res.q, target);
      EXPECT_LT(pos, 1e-4);
      EXPECT_LT(rot, 1e-3);
    }
  }
  // Python kin.ik 가까운 시작 97.3 % (export_log) 와 같은 수준
  EXPECT_GT(ok, 280);
}

TEST(RotationError, NearPi)
{
  const Eigen::Matrix3d r = Eigen::AngleAxisd(M_PI - 1e-6, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  const Eigen::Vector3d e = rotationError(r, Eigen::Matrix3d::Identity());
  EXPECT_NEAR(e.norm(), M_PI - 1e-6, 1e-6);
  EXPECT_NEAR(std::abs(e.normalized().dot(Eigen::Vector3d(1, 2, 3).normalized())), 1.0, 1e-6);
}

TEST(Wrap, MatchesPythonModulo)
{
  Vector6d q;
  q << 0.0, M_PI, -M_PI, 3 * M_PI / 2, -7.0, 10.0;
  const Vector6d w = wrap(q);
  // numpy: (q + pi) % (2 pi) - pi
  EXPECT_NEAR(w[1], -M_PI, 1e-12);
  EXPECT_NEAR(w[2], -M_PI, 1e-12);
  EXPECT_NEAR(w[3], -M_PI / 2, 1e-12);
  EXPECT_NEAR(w[4], -7.0 + 2 * M_PI, 1e-12);
  EXPECT_NEAR(w[5], 10.0 - 4 * M_PI, 1e-12);
}

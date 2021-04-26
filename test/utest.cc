#include "types.h"

#include <gtest/gtest.h>

TEST(SE3, SE3ToTwist) {
  SE3 pose, pose_rec;
  Eigen::Matrix<double, 6, 1> twist;

  // identity R
  pose.R.setIdentity();
  pose.t << 0.1, 0.2, 0.3;
  twist = pose.toTwist();
  pose_rec = SE3::fromTwist(twist);

  EXPECT_TRUE(pose.t.isApprox(twist.head<3>()));
  EXPECT_TRUE(pose_rec.R.isApprox(pose.R));
  EXPECT_TRUE(pose_rec.t.isApprox(pose.t));

  // non-identity R
  pose.R = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
  twist = pose.toTwist();
  pose_rec = SE3::fromTwist(twist);

  EXPECT_TRUE(pose_rec.R.isApprox(pose.R));
  EXPECT_TRUE(pose_rec.t.isApprox(pose.t));
}

TEST(SE3, TwistToSE3) {
  Eigen::Matrix<double, 6, 1> twist, twist_rec;
  SE3 pose;

  // omega = 0
  twist.tail<3>().setZero();
  twist.head<3>() << 0.1, 0.2, 0.3;
  pose = SE3::fromTwist(twist);
  twist_rec = pose.toTwist();

  EXPECT_TRUE(pose.t.isApprox(twist.head<3>()));
  EXPECT_TRUE(twist_rec.isApprox(twist));

  // omega != 0
  twist << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  pose = SE3::fromTwist(twist);
  twist_rec = pose.toTwist();

  EXPECT_TRUE(twist_rec.isApprox(twist));
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}

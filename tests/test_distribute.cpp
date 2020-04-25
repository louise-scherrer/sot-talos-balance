#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "sot/core/parameter-server.hh"
#include "sot/talos_balance/distribute-wrench.hh"

#include <boost/test/unit_test.hpp>

#include "test-paths.h"

using namespace dynamicgraph::sot;
using namespace dynamicgraph::sot::talos_balance;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_distribute) {
  // --- General ---
  std::cout << "--- General ---" << std::endl;

  const double dt = 0.001;
  const std::string robot_name = "robot";

  Eigen::VectorXd q(39);

  q << 0.0, 0.0, 1.018213, 0.0, 0.0, 0.0, 1.0,                        // Free flyer
      0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.0,                 // Left Leg
      0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.0,                 // Right Leg
      0.0, 0.006761,                                                  // Chest
      0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005,  // Left Arm
      -0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005,  // Right Arm
      0., 0.;                                                         // Head

  std::cout << "q: " << q.transpose() << std::endl;

  std::string urdfPath = TALOS_DATA_MODEL_DIR "urdf/talos_reduced.urdf";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data(model);
  Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  double m = data.mass[0];
  // com[1] = 0. # ensure perfect symmetry

  std::cout << "com: " << com.transpose() << std::endl;

  const std::string leftName = "leg_left_6_joint";
  const pinocchio::FrameIndex leftId = model.getFrameId(leftName);
  pinocchio::SE3 leftPos = data.oMf[leftId];
  std::cout << leftName << ": " << leftId << std::endl;
  std::cout << leftPos << std::endl;

  const std::string rightName = "leg_right_6_joint";
  const pinocchio::FrameIndex rightId = model.getFrameId(rightName);
  pinocchio::SE3 rightPos = data.oMf[rightId];
  // Eigen::Vector3d pR = leftPos.translation // ensure perfect symmetry
  // pR[1] = -pR[1]
  // rightPos.translation = pR
  std::cout << rightName << ": " << rightId << std::endl;
  std::cout << rightPos << std::endl;

  double g = 9.81;
  double fz = m * g;
  Eigen::Vector3d force(0.0, 0.0, fz);
  double lx = com[0];
  double tauy = -fz * lx;
  Eigen::Vector3d moment(0.0, tauy, 0.0);
  Eigen::VectorXd wrench(6);
  wrench << force, moment;

  std::cout << "total wrench: " << wrench.transpose() << std::endl;

  // --- Parameter server ---
  std::cout << "--- Parameter server ---" << std::endl;

  ParameterServer param_server("param_server");

  param_server.init(dt, urdfPath, robot_name);

  // Set the map from joint name to joint ID
  // for key in conf.mapJointNameToID:
  //      param_server.setNameToId(key, conf.mapJointNameToID[key]);

  //    # Set the map joint limits for each id
  //    for key in conf.mapJointLimits:
  //        param_server.setJointLimitsFromId(key, conf.mapJointLimits[key][0],
  //                                          conf.mapJointLimits[key][1])

  //    # Set the force limits for each id
  //    for key in conf.mapForceIdToForceLimits:
  //        param_server.setForceLimitsFromId(key, tuple(conf.mapForceIdToForceLimits[key][0]),
  //                                          tuple(conf.mapForceIdToForceLimits[key][1]))

  // Set the force sensor id for each sensor name
  //    for key in conf.mapNameToForceId:
  //        param_server.setForceNameToForceId(key, conf.mapNameToForceId[key])

  //    # Set the map from the urdf joint list to the sot joint list
  //    param_server.setJointsUrdfToSot(conf.urdftosot)

  // Set the foot frame name
  param_server.setFootFrameName("Right", "leg_right_6_joint");
  param_server.setFootFrameName("Left", "leg_left_6_joint");

  //    # Set IMU hosting joint name
  //    param_server.setImuJointName(conf.ImuJointName)

  Eigen::VectorXd rightFootSensorXYZ(3);
  rightFootSensorXYZ << 0.0, 0.0, -0.085;
  param_server.setRightFootForceSensorXYZ(rightFootSensorXYZ);

  Eigen::VectorXd rightFootSoleXYZ(3);
  rightFootSoleXYZ << 0.0, 0.0, -0.105;

  param_server.setRightFootSoleXYZ(rightFootSoleXYZ);

  // --- DistributeWrench ---
  std::cout << "--- DistributeWrench ---" << std::endl;

  DistributeWrench distribute("distribute");

  distribute.m_phaseSIN.setConstant(0);
  distribute.m_rhoSIN.setConstant(0.5);

  Eigen::VectorXd RIGHT_FOOT_SIZES(4);
  RIGHT_FOOT_SIZES << 0.100, -0.100, 0.06, -0.06;  // pos x, neg x, pos y, neg y size
  Eigen::VectorXd LEFT_FOOT_SIZES(4);
  LEFT_FOOT_SIZES << 0.100, -0.100, 0.06, -0.06;  // pos x, neg x, pos y, neg y size

  const double minPressure = 15.;
  const double frictionCoefficient = 0.7;

  const double wSum = 10000.0;
  const double wNorm = 10.0;
  const double wRatio = 1.0;
  Eigen::VectorXd wAnkle(6);
  wAnkle << 1., 1., 1e-4, 1., 1., 1e-4;

  distribute.m_eps = minPressure;  // setMinPressure(minPressure);
  distribute.m_frictionCoefficientSIN.setConstant(frictionCoefficient);
  distribute.m_wSumSIN.setConstant(wSum);
  distribute.m_wNormSIN.setConstant(wNorm);
  distribute.m_wRatioSIN.setConstant(wRatio);
  distribute.m_wAnkleSIN.setConstant(wAnkle);

  distribute.set_right_foot_sizes(RIGHT_FOOT_SIZES);
  distribute.set_left_foot_sizes(LEFT_FOOT_SIZES);

  distribute.m_qSIN.setConstant(q);
  distribute.m_wrenchDesSIN.setConstant(wrench);

  distribute.init(robot_name);

  // --- Wrench distribution ---
  std::cout << std::endl;
  std::cout << "--- Wrench distribution ---" << std::endl;
  distribute.m_phaseSIN.setConstant(0);

  Eigen::Vector3d forceLeft(0.0, 0.0, fz / 2);
  Eigen::Vector3d forceRight(0.0, 0.0, fz / 2);
  const double ly = leftPos.translation()[1];
  const double taux = fz * ly / 2;
  Eigen::Vector3d momentLeft(taux, tauy / 2, 0.0);
  Eigen::Vector3d momentRight(-taux, tauy / 2, 0.0);

  Eigen::VectorXd wrenchLeft(6);
  wrenchLeft << forceLeft, momentLeft;
  Eigen::VectorXd wrenchRight(6);
  wrenchRight << forceRight, momentRight;

  lx = com[0] - leftPos.translation()[0];
  tauy = -fz * lx / 2;
  Eigen::Vector3d ankleMomentLeft(0.0, tauy, 0.0);
  Eigen::Vector3d ankleMomentRight(0.0, tauy, 0.0);

  Eigen::VectorXd ankleWrenchLeft(6);
  ankleWrenchLeft << forceLeft, ankleMomentLeft;
  Eigen::VectorXd ankleWrenchRight(6);
  ankleWrenchRight << forceRight, ankleMomentRight;

  std::cout << "expected global wrench:       " << wrench.transpose() << std::endl;
  std::cout << "expected global left wrench:  " << wrenchLeft.transpose() << std::endl;
  std::cout << "expected global right wrench: " << wrenchRight.transpose() << std::endl;
  std::cout << "expected ankle left wrench:   " << ankleWrenchLeft.transpose() << std::endl;
  std::cout << "expected ankle right wrench:  " << ankleWrenchRight.transpose() << std::endl;

  Eigen::Vector3d copLeft(com[0] - leftPos.translation()[0], 0., 0.);
  Eigen::Vector3d copRight(com[0] - rightPos.translation()[0], 0., 0.);

  std::cout << "expected sole left CoP: " << copLeft.transpose() << std::endl;
  std::cout << "expected sole right CoP: " << copRight.transpose() << std::endl;
  std::cout << std::endl;

  distribute.m_zmpRefSOUT.recompute(0);

  std::cout << "resulting global wrench: " << distribute.m_wrenchRefSOUT(0).transpose() << std::endl;
  BOOST_CHECK(wrench.isApprox(distribute.m_wrenchRefSOUT(0), 1e-3));

  std::cout << "resulting global left wrench:  " << distribute.m_wrenchLeftSOUT(0).transpose() << std::endl;
  BOOST_CHECK(wrenchLeft.isApprox(distribute.m_wrenchLeftSOUT(0), 1e-3));
  std::cout << "resulting global right wrench: " << distribute.m_wrenchRightSOUT(0).transpose() << std::endl;
  BOOST_CHECK(wrenchRight.isApprox(distribute.m_wrenchRightSOUT(0), 1e-3));

  distribute.m_ankleWrenchLeftSOUT.recompute(0);
  distribute.m_ankleWrenchRightSOUT.recompute(0);

  std::cout << "resulting ankle left wrench:  " << distribute.m_ankleWrenchLeftSOUT(0).transpose() << std::endl;
  BOOST_CHECK(ankleWrenchLeft.isApprox(distribute.m_ankleWrenchLeftSOUT(0), 1e-3));
  std::cout << "resulting ankle right wrench: " << distribute.m_ankleWrenchRightSOUT(0).transpose() << std::endl;
  BOOST_CHECK(ankleWrenchRight.isApprox(distribute.m_ankleWrenchRightSOUT(0), 1e-3));

  distribute.m_copLeftSOUT.recompute(0);
  distribute.m_copRightSOUT.recompute(0);

  std::cout << "resulting sole left CoP:  " << distribute.m_copLeftSOUT(0).transpose() << std::endl;
  BOOST_CHECK(copLeft.isApprox(distribute.m_copLeftSOUT(0), 1e-3));
  std::cout << "resulting sole right CoP: " << distribute.m_copRightSOUT(0).transpose() << std::endl;
  BOOST_CHECK(copRight.isApprox(distribute.m_copRightSOUT(0), 1e-3));

  distribute.m_emergencyStopSOUT.recompute(0);
  bool stop = distribute.m_emergencyStopSOUT(0);
  BOOST_CHECK(!stop);

  // --- Wrench saturation ---
  std::cout << std::endl;
  std::cout << "--- Wrench saturation ---" << std::endl;
  std::cout << "NOTE: \"predicted\" wrench values are not accurate due to the foot saturation and as such they are "
               "not checked."
            << std::endl;
  std::cout
      << "CoP values are predicted under the assumption that they are at the foot border and as such they are checked."
      << std::endl;

  // --- Wrench saturation (left) ---
  std::cout << std::endl;
  std::cout << "--- Wrench saturation (left) ---" << std::endl;
  distribute.m_phaseSIN.setConstant(1);
  distribute.m_phaseSIN.setTime(1);

  wrenchLeft = wrench;
  ankleWrenchLeft = leftPos.actInv(pinocchio::Force(wrenchLeft)).toVector();

  std::cout << "expected global wrench: " << wrench.transpose() << std::endl;
  std::cout << "expected global left wrench: " << wrenchLeft.transpose() << std::endl;
  std::cout << "expected ankle left wrench: " << ankleWrenchLeft.transpose() << std::endl;

  copLeft << com[0] - leftPos.translation()[0], RIGHT_FOOT_SIZES[3], 0.;

  std::cout << "expected sole left CoP: " << copLeft.transpose() << std::endl;
  std::cout << std::endl;

  distribute.m_zmpRefSOUT.recompute(1);

  std::cout << "resulting global wrench: " << distribute.m_wrenchRefSOUT(1).transpose() << std::endl;
  // BOOST_CHECK(wrench.isApprox(distribute.m_wrenchRefSOUT(1)));
  std::cout << "resulting global left wrench: " << distribute.m_wrenchLeftSOUT(1).transpose() << std::endl;
  // BOOST_CHECK(wrenchLeft.isApprox(distribute.m_wrenchLeftSOUT(1)));

  distribute.m_ankleWrenchLeftSOUT.recompute(1);

  std::cout << "resulting ankle left wrench: " << distribute.m_ankleWrenchLeftSOUT(1).transpose() << std::endl;
  // BOOST_CHECK(ankleWrenchLeft.isApprox(distribute.m_ankleWrenchLeftSOUT(1)));

  distribute.m_copLeftSOUT.recompute(1);
  distribute.m_copRightSOUT.recompute(1);

  std::cout << "resulting sole left CoP: " << distribute.m_copLeftSOUT(1).transpose() << std::endl;
  BOOST_CHECK(copLeft.isApprox(distribute.m_copLeftSOUT(1), 1e-4));

  distribute.m_emergencyStopSOUT.recompute(1);
  stop = distribute.m_emergencyStopSOUT(1);
  BOOST_CHECK(!stop);

  // --- Wrench saturation (right) ---
  std::cout << std::endl;
  std::cout << "--- Wrench saturation (right) ---" << std::endl;
  distribute.m_phaseSIN.setConstant(-1);
  distribute.m_phaseSIN.setTime(2);

  wrenchRight = wrench;
  ankleWrenchRight = rightPos.actInv(pinocchio::Force(wrenchRight)).toVector();

  std::cout << "expected global wrench: " << wrench.transpose() << std::endl;
  std::cout << "expected global right wrench: " << wrenchRight.transpose() << std::endl;
  std::cout << "expected ankle right wrench: " << ankleWrenchRight.transpose() << std::endl;

  copRight << com[0] - rightPos.translation()[0], RIGHT_FOOT_SIZES[2], 0.;

  std::cout << "expected sole right CoP: " << copRight.transpose() << std::endl;
  std::cout << std::endl;

  distribute.m_zmpRefSOUT.recompute(2);

  std::cout << "resulting global wrench: " << distribute.m_wrenchRefSOUT(2).transpose() << std::endl;
  // BOOST_CHECK(wrench.isApprox(distribute.m_wrenchRefSOUT(2)));
  std::cout << "resulting global right wrench: " << distribute.m_wrenchRightSOUT(2).transpose() << std::endl;
  // BOOST_CHECK(wrenchRight.isApprox(distribute.m_wrenchRightSOUT(2)));

  distribute.m_ankleWrenchRightSOUT.recompute(2);

  std::cout << "resulting ankle right wrench: " << distribute.m_ankleWrenchRightSOUT(2).transpose() << std::endl;
  // BOOST_CHECK(ankleWrenchRight.isApprox(distribute.m_ankleWrenchRightSOUT(2)));

  distribute.m_copLeftSOUT.recompute(2);
  distribute.m_copRightSOUT.recompute(2);

  std::cout << "resulting sole right CoP: " << distribute.m_copRightSOUT(2).transpose() << std::endl;
  BOOST_CHECK(copRight.isApprox(distribute.m_copRightSOUT(2), 1e-4));

  distribute.m_emergencyStopSOUT.recompute(2);
  stop = distribute.m_emergencyStopSOUT(2);
  BOOST_CHECK(!stop);
}

BOOST_AUTO_TEST_SUITE_END()

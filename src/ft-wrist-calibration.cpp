/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 * F. Risbourg
 */

#include <sot/talos_balance/ft-wrist-calibration.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>

#define CALIB_ITER_TIME 1000  // Iteration needed for sampling and averaging the FT sensors while calibrating

using namespace sot::talos_balance;

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace dg::sot::talos_balance;

#define INPUT_SIGNALS m_rightWristForceInSIN << m_leftWristForceInSIN << m_qSIN
#define INNER_SIGNALS m_rightWeightSINNER << m_leftWeightSINNER
#define OUTPUT_SIGNALS m_rightWristForceOutSOUT << m_leftWristForceOutSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef FtWristCalibration EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FtWristCalibration, "FtWristCalibration");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
FtWristCalibration::FtWristCalibration(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(rightWristForceIn, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(leftWristForceIn, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_INNER(rightWeight, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(leftWeight, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(rightWristForceOut, dynamicgraph::Vector, INPUT_SIGNALS << INNER_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(leftWristForceOut, dynamicgraph::Vector, INPUT_SIGNALS << INNER_SIGNALS),
      m_robot_util(RefVoidRobotUtil()),
      m_model(),
      m_data(),
      m_rightSensorId(),
      m_leftSensorId(),
      m_initSucceeded(false),
      m_removeWeight(false) {
  Entity::signalRegistration(INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &FtWristCalibration::init,
                                      docCommandVoid1("Initialize the entity.", "Robot reference (string)")));
  addCommand("setRightHandConf",
             makeCommandVoid2(*this, &FtWristCalibration::setRightHandConf,
                              docCommandVoid2("Set the data of the right hand", "Vector of default forces in Newton",
                                              "Vector of the weight lever arm")));
  addCommand("setLeftHandConf",
             makeCommandVoid2(*this, &FtWristCalibration::setLeftHandConf,
                              docCommandVoid2("Set the data of the left hand", "Vector of default forces in Newton",
                                              "Vector of the weight lever arm")));
  addCommand("calibrateWristSensor", makeCommandVoid0(*this, &FtWristCalibration::calibrateWristSensor,
                                                      docCommandVoid0("Calibrate the wrist sensors")));

  addCommand("setRemoveWeight", makeCommandVoid1(*this, &FtWristCalibration::setRemoveWeight,
                                                 docCommandVoid1("set RemoveWeight", "desired removeWeight")));
}

void FtWristCalibration::init(const std::string &robotRef) {
  dgADD_OSTREAM_TO_RTLOG(std::cout);
  std::string localName(robotRef);
  if (!isNameInRobotUtil(localName)) {
    m_robot_util = createRobotUtil(localName);
  } else {
    m_robot_util = getRobotUtil(localName);
  }
  m_right_FT_offset << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset << 0, 0, 0, 0, 0, 0;
  m_right_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_right_weight_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_left_weight_calibration_sum << 0, 0, 0, 0, 0, 0;

  pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
  m_data = new pinocchio::Data(m_model);

  m_rightSensorId = m_model.getFrameId("wrist_right_ft_link");
  m_leftSensorId = m_model.getFrameId("wrist_left_ft_link");

  m_initSucceeded = true;
  SEND_MSG("Entity Initialized", MSG_TYPE_INFO);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_INNER_FUNCTION(rightWeight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal rightWeight before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Vector &q = m_qSIN(iter);
  assert(q.size() == m_model.nq && "Unexpected size of signal q");

  // Get sensorPlacement
  pinocchio::framesForwardKinematics(m_model, *m_data, q);
  const pinocchio::SE3 &sensorPlacement = m_data->oMf[m_rightSensorId];

  Eigen::Vector3d leverArm = sensorPlacement.rotation() * m_rightLeverArm;

  m_rightHandWeight[3] = leverArm(1) * m_rightHandWeight(2);
  m_rightHandWeight[4] = -leverArm(0) * m_rightHandWeight(2);

  Eigen::Matrix<double, 6, 1> weight;

  weight.head<3>() = sensorPlacement.rotation().transpose() * m_rightHandWeight.head<3>();
  weight.tail<3>() = sensorPlacement.rotation().transpose() * m_rightHandWeight.tail<3>();

  s = weight;

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(leftWeight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal rightWeight before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Vector &q = m_qSIN(iter);
  assert(q.size() == m_model.nq && "Unexpected size of signal q");

  pinocchio::framesForwardKinematics(m_model, *m_data, q);
  const pinocchio::SE3 &sensorPlacement = m_data->oMf[m_leftSensorId];

  Eigen::Vector3d leverArm = sensorPlacement.rotation() * m_leftLeverArm;

  m_leftHandWeight[3] = leverArm(1) * m_leftHandWeight(2);
  m_leftHandWeight[4] = -leverArm(0) * m_leftHandWeight(2);

  Eigen::Matrix<double, 6, 1> weight;

  weight.head<3>() = sensorPlacement.rotation().transpose() * m_leftHandWeight.head<3>();
  weight.tail<3>() = sensorPlacement.rotation().transpose() * m_leftHandWeight.tail<3>();

  s = weight;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rightWristForceOut, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);
  const Vector &rightWristForce = m_rightWristForceInSIN(iter);
  assert(rightWristForce.size() == 6 && "Unexpected size of signal rightWristForceIn, should be 6.");
  const Vector &rightWeight = m_rightWeightSINNER(iter);
  assert(rightWeight.size() == 6 && "Unexpected size of signal rightWeight, should be 6.");

  // do offset calibration if needed
  if (m_rightCalibrationIter > 0) {
    m_right_FT_offset_calibration_sum += rightWristForce;
    m_right_weight_calibration_sum += rightWeight;
    m_rightCalibrationIter--;
  } else if (m_rightCalibrationIter == 0) {
    SEND_INFO_STREAM_MSG("Calibrating ft sensors...");
    m_right_FT_offset = m_right_FT_offset_calibration_sum / CALIB_ITER_TIME;
    m_right_FT_offset -= m_right_weight_calibration_sum / CALIB_ITER_TIME;
    m_rightCalibrationIter--;
  }

  s = rightWristForce - m_right_FT_offset;

  if (m_removeWeight) {
    s -= rightWeight;
  }
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(leftWristForceOut, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);
  const Vector &leftWristForce = m_leftWristForceInSIN(iter);
  assert(leftWristForce.size() == 6 && "Unexpected size of signal leftWristForceIn, should be 6.");
  const Vector &leftWeight = m_leftWeightSINNER(iter);
  assert(leftWeight.size() == 6 && "Unexpected size of signal leftWeight, should be 6.");

  // do offset calibration if needed
  if (m_leftCalibrationIter > 0) {
    m_left_FT_offset_calibration_sum += leftWristForce;
    m_left_weight_calibration_sum += leftWeight;
    m_leftCalibrationIter--;
  } else if (m_leftCalibrationIter == 0) {
    m_left_FT_offset = m_left_FT_offset_calibration_sum / CALIB_ITER_TIME;
    m_left_FT_offset -= m_left_weight_calibration_sum / CALIB_ITER_TIME;
    m_leftCalibrationIter--;
  }
  // remove offset and hand weight
  s = leftWristForce - m_left_FT_offset;

  if (m_removeWeight) {
    s -= leftWeight;
  }
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

void FtWristCalibration::setRightHandConf(const double &rightW, const Vector &rightLeverArm) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set right hand weight before initialization!");
    return;
  }
  m_rightHandWeight << 0, 0, rightW, 0, 0, 0;
  m_rightLeverArm << rightLeverArm[0], rightLeverArm[1], rightLeverArm[2];
}

void FtWristCalibration::setLeftHandConf(const double &leftW, const Vector &leftLeverArm) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set left hand weight before initialization!");
    return;
  }
  m_leftHandWeight << 0, 0, leftW, 0, 0, 0;
  m_leftLeverArm << leftLeverArm[0], leftLeverArm[1], leftLeverArm[2];
}

void FtWristCalibration::calibrateWristSensor() {
  SEND_WARNING_STREAM_MSG(
      "Sampling FT sensor for offset calibration... Robot should be in the air, with horizontal hand.");
  m_rightCalibrationIter = CALIB_ITER_TIME;
  m_leftCalibrationIter = CALIB_ITER_TIME;
  m_right_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
}

void FtWristCalibration::setRemoveWeight(const bool &removeWeight) { m_removeWeight = removeWeight; }

/* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */
/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void FtWristCalibration::display(std::ostream &os) const {
  os << "FtWristCalibration " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

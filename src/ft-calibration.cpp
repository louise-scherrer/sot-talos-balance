/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 */

#include <sot/talos_balance/ft-calibration.hh>
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

#define INPUT_SIGNALS m_right_foot_force_inSIN << m_left_foot_force_inSIN
#define OUTPUT_SIGNALS m_right_foot_force_outSOUT << m_left_foot_force_outSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef FtCalibration EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FtCalibration, "FtCalibration");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
FtCalibration::FtCalibration(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(right_foot_force_in, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(left_foot_force_in, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(right_foot_force_out, dynamicgraph::Vector, m_right_foot_force_inSIN),
      CONSTRUCT_SIGNAL_OUT(left_foot_force_out, dynamicgraph::Vector, m_left_foot_force_inSIN),
      m_robot_util(RefVoidRobotUtil()),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &FtCalibration::init,
                                      docCommandVoid1("Initialize the entity.", "Robot reference (string)")));
  addCommand("setRightFootWeight",
             makeCommandVoid1(*this, &FtCalibration::setRightFootWeight,
                              docCommandVoid1("Set the weight of the right foot underneath the sensor",
                                              "Vector of default forces in Newton")));
  addCommand("setLeftFootWeight",
             makeCommandVoid1(*this, &FtCalibration::setLeftFootWeight,
                              docCommandVoid1("Set the weight of the left foot underneath the sensor",
                                              "Vector of default forces in Newton")));
  addCommand("calibrateFeetSensor", makeCommandVoid0(*this, &FtCalibration::calibrateFeetSensor,
                                                     docCommandVoid0("Calibrate the feet senors")));
}

void FtCalibration::init(const std::string &robotRef) {
  dgADD_OSTREAM_TO_RTLOG(std::cout);
  std::string localName(robotRef);
  m_initSucceeded = true;
  if (!isNameInRobotUtil(localName)) {
    m_robot_util = createRobotUtil(localName);
  } else {
    m_robot_util = getRobotUtil(localName);
  }
  m_right_FT_offset << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset << 0, 0, 0, 0, 0, 0;
  m_right_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  SEND_MSG("Entity Initialized", MSG_TYPE_INFO);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_force_out, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Vector &right_foot_force = m_right_foot_force_inSIN(iter);

  assert(right_foot_force.size() == 6 && "Unexpected size of signal right_foot_force_in, should be 6.");

  // do offset calibration if needed
  if (m_right_calibration_iter > 0) {
    m_right_FT_offset_calibration_sum += right_foot_force;
    m_right_calibration_iter--;
  } else if (m_right_calibration_iter == 0) {
    SEND_INFO_STREAM_MSG("Calibrating ft sensors...");
    m_right_FT_offset = m_right_FT_offset_calibration_sum / CALIB_ITER_TIME;  // todo copy
  }

  // remove offset and foot weight
  s = right_foot_force - m_left_foot_weight - m_right_FT_offset;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_force_out, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Vector &left_foot_force = m_left_foot_force_inSIN(iter);

  assert(left_foot_force.size() == 6 && "Unexpected size of signal left_foot_force_in, should be 6.");

  // do offset calibration if needed
  if (m_left_calibration_iter > 0) {
    m_left_FT_offset_calibration_sum += left_foot_force;
    m_left_calibration_iter--;
  } else if (m_left_calibration_iter == 0) {
    m_left_FT_offset = m_left_FT_offset_calibration_sum / CALIB_ITER_TIME;
  }
  // remove offset and foot weight
  s = left_foot_force - m_left_foot_weight - m_left_FT_offset;

  return s;
}
/* --- COMMANDS ---------------------------------------------------------- */

void FtCalibration::setRightFootWeight(const double &rightW) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set right foot weight before initialization!");
    return;
  }
  m_right_foot_weight << 0, 0, rightW, 0, 0, 0;
}

void FtCalibration::setLeftFootWeight(const double &leftW) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set left foot weight before initialization!");
    return;
  }
  m_left_foot_weight << 0, 0, leftW, 0, 0, 0;
}

void FtCalibration::calibrateFeetSensor() {
  SEND_WARNING_STREAM_MSG(
      "Sampling FT sensor for offset calibration... Robot should be in the air, with horizontal feet.");
  m_right_calibration_iter = CALIB_ITER_TIME;
  m_left_calibration_iter = CALIB_ITER_TIME;
  m_right_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
  m_left_FT_offset_calibration_sum << 0, 0, 0, 0, 0, 0;
}

/* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */
/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void FtCalibration::display(std::ostream &os) const {
  os << "FtCalibration " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

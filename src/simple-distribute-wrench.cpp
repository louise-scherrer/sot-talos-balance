/*
 * Copyright 2018, Gepetto team, LAAS-CNRS
 *
 * This file is part of sot-talos-balance.
 * sot-talos-balance is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-talos-balance is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-talos-balance.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sot/talos_balance/simple-distribute-wrench.hh"

#include <iostream>

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <sot/core/stop-watch.hh>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                                      "-------------------------------------------------------"
#define PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS \
  "SimpleDistributeWrench: kinematics computations              "
#define PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS \
  "SimpleDistributeWrench: wrenches computations                "

#define INPUT_SIGNALS m_wrenchDesSIN << m_qSIN << m_rhoSIN << m_phaseSIN

#define INNER_SIGNALS m_kinematics_computations << m_wrenches

#define OUTPUT_SIGNALS                                                                                        \
  m_wrenchLeftSOUT << m_ankleWrenchLeftSOUT << m_surfaceWrenchLeftSOUT << m_copLeftSOUT << m_wrenchRightSOUT  \
                   << m_ankleWrenchRightSOUT << m_surfaceWrenchRightSOUT << m_copRightSOUT << m_wrenchRefSOUT \
                   << m_zmpRefSOUT << m_emergencyStopSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleDistributeWrench EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleDistributeWrench, "SimpleDistributeWrench");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleDistributeWrench::SimpleDistributeWrench(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(wrenchDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rho, double),
      CONSTRUCT_SIGNAL_IN(phase, int),
      CONSTRUCT_SIGNAL_INNER(kinematics_computations, int, m_qSIN),
      CONSTRUCT_SIGNAL_INNER(wrenches, int,
                             m_wrenchDesSIN << m_rhoSIN << m_phaseSIN << m_kinematics_computationsSINNER),
      CONSTRUCT_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector, m_wrenchesSINNER),
      CONSTRUCT_SIGNAL_OUT(ankleWrenchLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(surfaceWrenchLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(copLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector, m_wrenchesSINNER),
      CONSTRUCT_SIGNAL_OUT(ankleWrenchRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(surfaceWrenchRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(copRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector, m_wrenchLeftSOUT << m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(zmpRef, dynamicgraph::Vector, m_wrenchRefSOUT),
      CONSTRUCT_SIGNAL_OUT(emergencyStop, bool, m_zmpRefSOUT),
      m_initSucceeded(false),
      m_model(),
      m_data(pinocchio::Model()) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &SimpleDistributeWrench::init,
                                      docCommandVoid1("Initialize the entity.", "Robot name")));
}

void SimpleDistributeWrench::init(const std::string& robotName) {
  if (!m_wrenchDesSIN.isPlugged()) return SEND_MSG("Init failed: signal wrenchDes is not plugged", MSG_TYPE_ERROR);
  if (!m_qSIN.isPlugged()) return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);

  try {
    /* Retrieve m_robot_util informations */
    std::string localName(robotName);
    if (isNameInRobotUtil(localName)) {
      m_robot_util = getRobotUtil(localName);
      //            std::cerr << "m_robot_util:" << m_robot_util << std::endl;
    } else {
      SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
      return;
    }

    pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
    m_data = pinocchio::Data(m_model);
  } catch (const std::exception& e) {
    std::cout << e.what();
    SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
    return;
  }

  assert(m_model.existFrame(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
  assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
  m_left_foot_id = m_model.getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
  m_right_foot_id = m_model.getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);

  // m_ankle_M_ftSens = pinocchio::SE3(Eigen::Matrix3d::Identity(),
  // m_robot_util->m_foot_util.m_Right_Foot_Force_Sensor_XYZ.head<3>());
  m_ankle_M_sole =
      pinocchio::SE3(Eigen::Matrix3d::Identity(), m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ.head<3>());

  m_initSucceeded = true;
}

Eigen::Vector3d SimpleDistributeWrench::computeCoP(const dg::Vector& wrenchGlobal, const pinocchio::SE3& pose) const {
  const pinocchio::Force::Vector6& wrench = pose.actInv(pinocchio::Force(wrenchGlobal)).toVector();

  const double h = pose.translation()[2];

  const double fx = wrench[0];
  const double fy = wrench[1];
  const double fz = wrench[2];
  const double tx = wrench[3];
  const double ty = wrench[4];

  double m_eps = 0.1;  // temporary

  double px, py;

  if (fz >= m_eps / 2) {
    px = (-ty - fx * h) / fz;
    py = (tx - fy * h) / fz;
  } else {
    px = 0.0;
    py = 0.0;
  }
  const double pz = 0.0;

  Eigen::Vector3d cop;
  cop[0] = px;
  cop[1] = py;
  cop[2] = pz;

  return cop;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_INNER_FUNCTION(kinematics_computations, int) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal kinematics_computations before initialization!");
    return s;
  }

  const Eigen::VectorXd& q = m_qSIN(iter);
  assert(q.size() == m_model.nq && "Unexpected size of signal q");

  getProfiler().start(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

  pinocchio::forwardKinematics(m_model, m_data, q);
  pinocchio::updateFramePlacement(m_model, m_data, m_left_foot_id);
  pinocchio::updateFramePlacement(m_model, m_data, m_right_foot_id);

  m_contactLeft = m_data.oMf[m_left_foot_id] * m_ankle_M_sole;
  m_contactRight = m_data.oMf[m_right_foot_id] * m_ankle_M_sole;

  getProfiler().stop(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

  return s;
}

void SimpleDistributeWrench::distributeWrench(const Eigen::VectorXd& wrenchDes, const double rho) {
  Eigen::Vector3d forceLeft = wrenchDes.head<3>() / 2;
  Eigen::Vector3d forceRight = forceLeft;
  forceLeft[2] = rho * wrenchDes[2];
  forceRight[2] = (1 - rho) * wrenchDes[2];

  Eigen::Vector3d tauLeft = m_contactLeft.translation().cross(forceLeft);
  Eigen::Vector3d tauRight = m_contactRight.translation().cross(forceRight);

  Eigen::Vector3d tauResidual = (wrenchDes.tail<3>() - tauLeft - tauRight) / 2;
  tauLeft += tauResidual;
  tauRight += tauResidual;

  m_wrenchLeft << forceLeft, tauLeft;
  m_wrenchRight << forceRight, tauRight;

  const bool success = true;

  m_emergency_stop_triggered = !success;
}

void SimpleDistributeWrench::saturateWrench(const Eigen::VectorXd& wrenchDes, const int phase) {
  const bool success = true;

  m_emergency_stop_triggered = !success;

  const Eigen::VectorXd& result = wrenchDes;

  if (phase > 0) {
    m_wrenchLeft = result;
    m_wrenchRight.setZero(6);
  } else if (phase < 0) {
    m_wrenchRight = result;
    m_wrenchLeft.setZero(6);
  }
}

DEFINE_SIGNAL_INNER_FUNCTION(wrenches, int) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal wrenches before initialization!");
    return s;
  }

  const Eigen::VectorXd& wrenchDes = m_wrenchDesSIN(iter);
  const int& dummy = m_kinematics_computationsSINNER(iter);
  (void)dummy;
  const int& phase = m_phaseSIN(iter);

  assert(wrenchDes.size() == 6 && "Unexpected size of signal wrenchDes");

  getProfiler().start(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS);

  if (phase == 0) {
    const double& rho = m_rhoSIN(iter);

    distributeWrench(wrenchDes, rho);
  } else {
    saturateWrench(wrenchDes, phase);
  }

  getProfiler().stop(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS);

  if (m_emergency_stop_triggered) {
    SEND_WARNING_STREAM_MSG("Error in wrench distribution!");
    return s;
  }

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(wrenchLeft, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchLeft before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const int& dummy = m_wrenchesSINNER(iter);
  (void)dummy;
  s = m_wrenchLeft;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(wrenchRight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchRight before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const int& dummy = m_wrenchesSINNER(iter);
  (void)dummy;
  s = m_wrenchRight;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(ankleWrenchLeft, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal ankleWrenchLeft before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Eigen::VectorXd& wrenchLeft = m_wrenchLeftSOUT(iter);

  s = m_data.oMf[m_left_foot_id].actInv(pinocchio::Force(wrenchLeft)).toVector();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(ankleWrenchRight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal ankleWrenchRight before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Eigen::VectorXd& wrenchRight = m_wrenchRightSOUT(iter);

  s = m_data.oMf[m_right_foot_id].actInv(pinocchio::Force(wrenchRight)).toVector();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(surfaceWrenchLeft, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal surfaceWrenchLeft before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Eigen::VectorXd& wrenchLeft = m_wrenchLeftSOUT(iter);

  s = m_contactLeft.actInv(pinocchio::Force(wrenchLeft)).toVector();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(surfaceWrenchRight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal surfaceWrenchRight before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Eigen::VectorXd& wrenchRight = m_wrenchRightSOUT(iter);

  s = m_contactRight.actInv(pinocchio::Force(wrenchRight)).toVector();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(copLeft, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal copLeft before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Eigen::VectorXd& wrenchLeft = m_wrenchLeftSOUT(iter);

  if (m_emergency_stop_triggered) {
    s.setZero(3);
    return s;
  }

  s = computeCoP(wrenchLeft, m_contactLeft);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(copRight, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal copRight before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Eigen::VectorXd& wrenchRight = m_wrenchRightSOUT(iter);

  if (m_emergency_stop_triggered) {
    s.setZero(3);
    return s;
  }

  s = computeCoP(wrenchRight, m_contactRight);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(wrenchRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchRef before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Eigen::VectorXd& wrenchLeft = m_wrenchLeftSOUT(iter);
  const Eigen::VectorXd& wrenchRight = m_wrenchRightSOUT(iter);

  s = wrenchLeft + wrenchRight;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmpRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmpRef before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Eigen::VectorXd& wrenchRef = m_wrenchRefSOUT(iter);

  if (m_emergency_stop_triggered) {
    s.setZero(3);
    return s;
  }

  // const double fx = wrenchRef[0];
  // const double fy = wrenchRef[1];
  const double fz = wrenchRef[2];
  const double tx = wrenchRef[3];
  const double ty = wrenchRef[4];

  double m_eps = 0.1;  // temporary

  double px, py;
  if (fz >= m_eps / 2) {
    px = -ty / fz;
    py = tx / fz;
  } else {
    px = 0.0;
    py = 0.0;
  }
  const double pz = 0.0;

  Eigen::Vector3d zmp(3);
  zmp[0] = px;
  zmp[1] = py;
  zmp[2] = pz;

  s = zmp;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(emergencyStop, bool) {
  const dynamicgraph::Vector& zmp = m_zmpRefSOUT(iter);  // dummy to trigger zmp computation
  (void)zmp;                                             // disable unused variable warning
  s = m_emergency_stop_triggered;
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleDistributeWrench::display(std::ostream& os) const {
  os << "SimpleDistributeWrench " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

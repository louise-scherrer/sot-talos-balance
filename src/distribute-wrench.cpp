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

#include "sot/talos_balance/distribute-wrench.hh"

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
using namespace eiquadprog::solvers;

// Size to be aligned                                      "-------------------------------------------------------"
#define PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS "DistributeWrench: kinematics computations              "
#define PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS "DistributeWrench: QP problem computations              "

#define WEIGHT_SIGNALS m_wSumSIN << m_wNormSIN << m_wRatioSIN << m_wAnkleSIN

#define INPUT_SIGNALS m_wrenchDesSIN << m_qSIN << m_rhoSIN << m_phaseSIN << m_frictionCoefficientSIN << WEIGHT_SIGNALS

#define INNER_SIGNALS m_kinematics_computations << m_qp_computations

#define OUTPUT_SIGNALS                                                                                        \
  m_wrenchLeftSOUT << m_ankleWrenchLeftSOUT << m_surfaceWrenchLeftSOUT << m_copLeftSOUT << m_wrenchRightSOUT  \
                   << m_ankleWrenchRightSOUT << m_surfaceWrenchRightSOUT << m_copRightSOUT << m_wrenchRefSOUT \
                   << m_zmpRefSOUT << m_emergencyStopSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DistributeWrench EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DistributeWrench, "DistributeWrench");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
DistributeWrench::DistributeWrench(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(wrenchDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rho, double),
      CONSTRUCT_SIGNAL_IN(phase, int),
      CONSTRUCT_SIGNAL_IN(frictionCoefficient, double),
      CONSTRUCT_SIGNAL_IN(wSum, double),
      CONSTRUCT_SIGNAL_IN(wNorm, double),
      CONSTRUCT_SIGNAL_IN(wRatio, double),
      CONSTRUCT_SIGNAL_IN(wAnkle, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_INNER(kinematics_computations, int, m_qSIN),
      CONSTRUCT_SIGNAL_INNER(
          qp_computations, int,
          m_wrenchDesSIN << m_rhoSIN << m_phaseSIN << WEIGHT_SIGNALS << m_kinematics_computationsSINNER),
      CONSTRUCT_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector, m_qp_computationsSINNER),
      CONSTRUCT_SIGNAL_OUT(ankleWrenchLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(surfaceWrenchLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(copLeft, dynamicgraph::Vector, m_wrenchLeftSOUT),
      CONSTRUCT_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector, m_qp_computationsSINNER),
      CONSTRUCT_SIGNAL_OUT(ankleWrenchRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(surfaceWrenchRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(copRight, dynamicgraph::Vector, m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector, m_wrenchLeftSOUT << m_wrenchRightSOUT),
      CONSTRUCT_SIGNAL_OUT(zmpRef, dynamicgraph::Vector, m_wrenchRefSOUT),
      CONSTRUCT_SIGNAL_OUT(emergencyStop, bool, m_zmpRefSOUT),
      m_initSucceeded(false),
      m_model(),
      m_data(pinocchio::Model()),
      m_Q1(6, 6),
      m_C1(6),
      m_Aeq1(0, 6),
      m_Beq1(0),
      m_Aineq1(16, 6),
      m_Bineq1(16),
      m_result1(6),
      m_Q2(12, 12),
      m_C2(12),
      m_Aeq2(0, 12),
      m_Beq2(0),
      m_Aineq2(34, 12),
      m_Bineq2(34),
      m_result2(12),
      m_wAnkle(6) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  m_qp1.reset(6, 0, 16);
  m_qp2.reset(12, 0, 34);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &DistributeWrench::init,
                                      docCommandVoid1("Initialize the entity.", "Robot name")));

  addCommand(
      "set_right_foot_sizes",
      makeCommandVoid1(*this, &DistributeWrench::set_right_foot_sizes,
                       docCommandVoid1("Set the size of the right foot (pos x, neg x, pos y, neg y)", "4d vector")));
  addCommand(
      "set_left_foot_sizes",
      makeCommandVoid1(*this, &DistributeWrench::set_left_foot_sizes,
                       docCommandVoid1("Set the size of the left foot (pos x, neg x, pos y, neg y)", "4d vector")));

  addCommand("getMinPressure", makeDirectGetter(*this, &m_eps, docDirectGetter("Get minimum pressure", "double")));
  addCommand("setMinPressure", makeDirectSetter(*this, &m_eps, docDirectSetter("Set minimum pressure", "double")));

  m_eps = 15.;  // TODO: signal/conf
}

void DistributeWrench::init(const std::string& robotName) {
  if (!m_wrenchDesSIN.isPlugged()) return SEND_MSG("Init failed: signal wrenchDes is not plugged", MSG_TYPE_ERROR);
  if (!m_qSIN.isPlugged()) return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);

  if (m_left_foot_sizes.size() == 0) return SEND_ERROR_STREAM_MSG("Init failed: left foot size is not initialized");
  if (m_right_foot_sizes.size() == 0) return SEND_ERROR_STREAM_MSG("Init failed: right foot size is not initialized");

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

void DistributeWrench::set_right_foot_sizes(const dynamicgraph::Vector& s) {
  if (s.size() != 4) return SEND_MSG("Foot size vector should have size 4, not " + toString(s.size()), MSG_TYPE_ERROR);
  m_right_foot_sizes = s;
}

void DistributeWrench::set_left_foot_sizes(const dynamicgraph::Vector& s) {
  if (s.size() != 4) return SEND_MSG("Foot size vector should have size 4, not " + toString(s.size()), MSG_TYPE_ERROR);
  m_left_foot_sizes = s;
}

// WARNING: we are assuming wrench = right = symmetrical
void DistributeWrench::computeWrenchFaceMatrix(const double mu) {
  const double X = m_right_foot_sizes[0];
  const double Y = m_right_foot_sizes[2];
  m_wrenchFaceMatrix <<
      // fx,  fy,            fz,  mx,  my,  mz,
      -1,
      0, -mu, 0, 0, 0, +1, 0, -mu, 0, 0, 0, 0, -1, -mu, 0, 0, 0, 0, +1, -mu, 0, 0, 0, 0, 0, -Y, -1, 0, 0, 0, 0, -Y, +1,
      0, 0, 0, 0, -X, 0, -1, 0, 0, 0, -X, 0, +1, 0, -Y, -X, -(X + Y) * mu, +mu, +mu, -1, -Y, +X, -(X + Y) * mu, +mu,
      -mu, -1, +Y, -X, -(X + Y) * mu, -mu, +mu, -1, +Y, +X, -(X + Y) * mu, -mu, -mu, -1, +Y, +X, -(X + Y) * mu, +mu,
      +mu, +1, +Y, -X, -(X + Y) * mu, +mu, -mu, +1, -Y, +X, -(X + Y) * mu, -mu, +mu, +1, -Y, -X, -(X + Y) * mu, -mu,
      -mu, +1;
}

Eigen::Vector3d DistributeWrench::computeCoP(const dg::Vector& wrenchGlobal, const pinocchio::SE3& pose) const {
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

  getProfiler().start(PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

  pinocchio::forwardKinematics(m_model, m_data, q);
  pinocchio::updateFramePlacement(m_model, m_data, m_left_foot_id);
  pinocchio::updateFramePlacement(m_model, m_data, m_right_foot_id);

  m_contactLeft = m_data.oMf[m_left_foot_id] * m_ankle_M_sole;
  m_contactRight = m_data.oMf[m_right_foot_id] * m_ankle_M_sole;

  getProfiler().stop(PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

  return s;
}

void DistributeWrench::distributeWrench(const Eigen::VectorXd& wrenchDes, const double rho, const double mu) {
  // --- COSTS

  // Initialize cost matrices
  Eigen::MatrixXd& Q = m_Q2;
  Eigen::VectorXd& C = m_C2;

  // min |wrenchLeft + wrenchRight - wrenchDes|^2
  Q.topLeftCorner<6, 6>().setIdentity();
  Q.topRightCorner<6, 6>().setIdentity();
  Q.bottomLeftCorner<6, 6>().setIdentity();
  Q.bottomRightCorner<6, 6>().setIdentity();
  Q *= m_wSum;

  C.head<6>() = -wrenchDes;
  C.tail<6>() = -wrenchDes;
  C *= m_wSum;

  // min |wrenchLeft_a|^2 + |wrenchRight_a|^2
  Eigen::Matrix<double, 6, 6> tmp = m_wAnkle.asDiagonal() * m_data.oMf[m_left_foot_id].inverse().toDualActionMatrix();
  Q.topLeftCorner<6, 6>().noalias() += tmp.transpose() * tmp * m_wNorm;

  tmp = m_wAnkle.asDiagonal() * m_data.oMf[m_right_foot_id].inverse().toDualActionMatrix();
  Q.bottomRightCorner<6, 6>().noalias() += tmp.transpose() * tmp * m_wNorm;

  // min |(1-rho)e_z^T*wrenchLeft_c - rho*e_z^T*wrenchLeft_c|
  Eigen::Matrix<double, 1, 12> tmp2;
  tmp2 << (1 - rho) * (m_contactLeft.inverse().toDualActionMatrix().row(2)),
      (-rho) * (m_contactRight.inverse().toDualActionMatrix().row(2));

  Q.noalias() += tmp2.transpose() * tmp2 * m_wRatio;

  // --- Equality constraints

  Eigen::MatrixXd& Aeq = m_Aeq2;

  Eigen::VectorXd& Beq = m_Beq2;

  // --- Inequality constraints

  computeWrenchFaceMatrix(mu);

  Eigen::MatrixXd& Aineq = m_Aineq2;

  Aineq.topLeftCorner<16, 6>() = m_wrenchFaceMatrix * m_contactLeft.inverse().toDualActionMatrix();
  Aineq.topRightCorner<16, 6>().setZero();
  Aineq.block<16, 6>(16, 0).setZero();
  Aineq.block<16, 6>(16, 6) = m_wrenchFaceMatrix * m_contactRight.inverse().toDualActionMatrix();

  Aineq.block<1, 6>(32, 0) = -m_contactLeft.inverse().toDualActionMatrix().row(2);
  Aineq.block<1, 6>(32, 6).setZero();
  Aineq.block<1, 6>(33, 0).setZero();
  Aineq.block<1, 6>(33, 6) = -m_contactRight.inverse().toDualActionMatrix().row(2);

  Eigen::VectorXd& Bineq = m_Bineq2;

  Bineq.setZero();
  Bineq(32) = -m_eps;
  Bineq(33) = -m_eps;

  Eigen::VectorXd& result = m_result2;

  EiquadprogFast_status status = m_qp2.solve_quadprog(Q, C, Aeq, -Beq, -Aineq, Bineq, result);

  m_emergency_stop_triggered = (status != EIQUADPROG_FAST_OPTIMAL);

  if (m_emergency_stop_triggered) {
    m_wrenchLeft.setZero(6);
    m_wrenchRight.setZero(6);
  } else {
    m_wrenchLeft = result.head<6>();
    m_wrenchRight = result.tail<6>();
  }
}

void DistributeWrench::saturateWrench(const Eigen::VectorXd& wrenchDes, const int phase, const double mu) {
  // Initialize cost matrices
  Eigen::MatrixXd& Q = m_Q1;
  Eigen::VectorXd& C = m_C1;

  // min |wrench - wrenchDes|^2
  Q.setIdentity();
  C = -wrenchDes;

  // --- Equality constraints

  Eigen::MatrixXd& Aeq = m_Aeq1;

  Eigen::VectorXd& Beq = m_Beq1;

  // --- Inequality constraints

  computeWrenchFaceMatrix(mu);

  Eigen::MatrixXd& Aineq = m_Aineq1;
  if (phase > 0) {
    Aineq = m_wrenchFaceMatrix * m_contactLeft.inverse().toDualActionMatrix();
  } else {
    Aineq = m_wrenchFaceMatrix * m_contactRight.inverse().toDualActionMatrix();
  }

  Eigen::VectorXd& Bineq = m_Bineq1;
  Bineq.setZero();

  Eigen::VectorXd& result = m_result1;

  EiquadprogFast_status status = m_qp1.solve_quadprog(Q, C, Aeq, -Beq, -Aineq, Bineq, result);

  m_emergency_stop_triggered = (status != EIQUADPROG_FAST_OPTIMAL);

  if (m_emergency_stop_triggered) {
    m_wrenchLeft.setZero(6);
    m_wrenchRight.setZero(6);
  } else {
    if (phase > 0) {
      m_wrenchLeft = result;
      m_wrenchRight.setZero(6);
    } else if (phase < 0) {
      m_wrenchRight = result;
      m_wrenchLeft.setZero(6);
    }
  }
}

DEFINE_SIGNAL_INNER_FUNCTION(qp_computations, int) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal qp_computations before initialization!");
    return s;
  }

  const Eigen::VectorXd& wrenchDes = m_wrenchDesSIN(iter);
  const int& dummy = m_kinematics_computationsSINNER(iter);
  (void)dummy;
  const int& phase = m_phaseSIN(iter);

  const double& mu = m_frictionCoefficientSIN(iter);  // 0.7

  assert(wrenchDes.size() == 6 && "Unexpected size of signal wrenchDes");

  getProfiler().start(PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS);

  if (phase == 0) {
    const double& rho = m_rhoSIN(iter);

    m_wSum = m_wSumSIN(iter);      // 10000.0
    m_wNorm = m_wNormSIN(iter);    // 10.0
    m_wRatio = m_wRatioSIN(iter);  // 1.0
    m_wAnkle = m_wAnkleSIN(iter);  // 1., 1., 1e-4, 1., 1., 1e-4

    distributeWrench(wrenchDes, rho, mu);
  } else {
    saturateWrench(wrenchDes, phase, mu);
  }

  getProfiler().stop(PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS);

  if (m_emergency_stop_triggered) {
    SEND_ERROR_STREAM_MSG("No solution to the QP problem!");
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

  const int& dummy = m_qp_computationsSINNER(iter);
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

  const int& dummy = m_qp_computationsSINNER(iter);
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

void DistributeWrench::display(std::ostream& os) const {
  os << "DistributeWrench " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

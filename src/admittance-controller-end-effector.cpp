/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Fanny Risbourg
 * This file is part of sot-talos-balance.
 * See license file.
 */

#include "sot/talos_balance/admittance-controller-end-effector.hh"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace pinocchio;
using namespace dg::command;

#define PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WFORCE_COMPUTATION \
  "AdmittanceControllerEndEffector: w_force computation   "

#define PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WDQ_COMPUTATION \
  "AdmittanceControllerEndEffector: w_dq computation      "

#define PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQ_COMPUTATION \
  "AdmittanceControllerEndEffector: dq computation        "

#define INPUT_SIGNALS m_KpSIN << m_KdSIN << m_dqSaturationSIN << m_forceSIN << m_w_forceDesSIN << m_qSIN

#define INNER_SIGNALS m_w_forceSINNER << m_w_dqSINNER

#define OUTPUT_SIGNALS m_dqSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef AdmittanceControllerEndEffector EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AdmittanceControllerEndEffector, "AdmittanceControllerEndEffector");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
AdmittanceControllerEndEffector::AdmittanceControllerEndEffector(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Kd, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dqSaturation, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(force, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(w_forceDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_INNER(w_force, dynamicgraph::Vector, m_forceSIN),
      CONSTRUCT_SIGNAL_INNER(w_dq, dynamicgraph::Vector, INPUT_SIGNALS << m_w_forceSINNER),
      CONSTRUCT_SIGNAL_OUT(dq, dynamicgraph::Vector, m_w_dqSINNER),
      m_initSucceeded(false),
      m_robot_util(),
      m_model(),
      m_data(),
      m_sensorFrameId(),
      m_endEffectorId() {
  Entity::signalRegistration(INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid3(*this, &AdmittanceControllerEndEffector::init,
                                      docCommandVoid3("Initialize the entity.", "time step", "sensor frame name",
                                                      "end Effector Joint Name")));
  addCommand("resetDq",
             makeCommandVoid0(*this, &AdmittanceControllerEndEffector::resetDq, docCommandVoid0("resetDq")));
}

void AdmittanceControllerEndEffector::init(const double &dt, const std::string &sensorFrameName,
                                           const std::string &endEffectorName) {
  if (!m_dqSaturationSIN.isPlugged())
    return SEND_MSG("Init failed: signal dqSaturation is not plugged", MSG_TYPE_ERROR);
  if (!m_KpSIN.isPlugged()) return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_KdSIN.isPlugged()) return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
  if (!m_forceSIN.isPlugged()) return SEND_MSG("Init failed: signal force is not plugged", MSG_TYPE_ERROR);
  if (!m_w_forceDesSIN.isPlugged()) return SEND_MSG("Init failed: signal w_forceDes is not plugged", MSG_TYPE_ERROR);
  if (!m_qSIN.isPlugged()) return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);

  m_n = 6;
  m_dt = dt;
  m_w_dq.setZero(m_n);

  try {
    /* Retrieve m_robot_util informations */
    std::string localName("robot");
    if (isNameInRobotUtil(localName)) {
      m_robot_util = getRobotUtil(localName);
      std::cerr << "m_robot_util:" << m_robot_util << std::endl;
    } else {
      SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
      return;
    }

    pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
    m_data = new pinocchio::Data(m_model);

    m_endEffectorId = m_model.getJointId(endEffectorName);
    m_sensorFrameId = m_model.getFrameId(sensorFrameName);
  } catch (const std::exception &e) {
    std::cout << e.what();
    SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
    return;
  }

  m_initSucceeded = true;
}

void AdmittanceControllerEndEffector::resetDq() {
  m_w_dq.setZero(m_n);
  return;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */
DEFINE_SIGNAL_INNER_FUNCTION(w_force, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal w_force before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WFORCE_COMPUTATION);

  const Vector &force = m_forceSIN(iter);
  const Vector &q = m_qSIN(iter);
  assert(force.size() == m_n && "Unexpected size of signal force");
  assert(q.size() == m_model.nq && "Unexpected size of signal q");

  // Get sensorPlacement
  pinocchio::framesForwardKinematics(m_model, *m_data, q);
  pinocchio::SE3 sensorPlacement = m_data->oMf[m_sensorFrameId];

  s = sensorPlacement.act(pinocchio::Force(force)).toVector();

  getProfiler().stop(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WFORCE_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(w_dq, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal w_dq before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WDQ_COMPUTATION);

  const Vector &w_forceDes = m_w_forceDesSIN(iter);
  const Vector &w_force = m_w_forceSINNER(iter);
  const Vector &Kp = m_KpSIN(iter);
  const Vector &Kd = m_KdSIN(iter);
  const Vector &dqSaturation = m_dqSaturationSIN(iter);
  assert(w_force.size() == m_n && "Unexpected size of signal force");
  assert(w_forceDes.size() == m_n && "Unexpected size of signal w_forceDes");
  assert(Kp.size() == m_n && "Unexpected size of signal Kp");
  assert(Kd.size() == m_n && "Unexpected size of signal Kd");
  assert(dqSaturation.size() == m_n && "Unexpected size of signal dqSaturation");

  m_w_dq = m_w_dq + m_dt * (Kp.cwiseProduct(w_forceDes - w_force)) - Kd.cwiseProduct(m_w_dq);

  for (int i = 0; i < m_n; i++) {
    if (m_w_dq[i] > dqSaturation[i]) m_w_dq[i] = dqSaturation[i];
    if (m_w_dq[i] < -dqSaturation[i]) m_w_dq[i] = -dqSaturation[i];
  }

  s = m_w_dq;

  getProfiler().stop(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_WDQ_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dq, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dq before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQ_COMPUTATION);

  const Vector &w_dq = m_w_dqSINNER(iter);
  assert(w_dq.size() == m_n && "Unexpected size of signal w_dq");

  // Get endEffectorPlacement
  pinocchio::SE3 placement = m_data->oMi[m_endEffectorId];

  s = placement.actInv(pinocchio::Motion(w_dq)).toVector();

  getProfiler().stop(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQ_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */
void AdmittanceControllerEndEffector::display(std::ostream &os) const {
  os << "AdmittanceControllerEndEffector " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

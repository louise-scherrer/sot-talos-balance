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

#include "sot/talos_balance/com-admittance-controller.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                                         "-------------------------------------------------------"
#define PROFILE_COMADMITTANCECONTROLLER_DDCOMREF_COMPUTATION "ComAdmittanceController: ddcomRef computation          "
#define PROFILE_COMADMITTANCECONTROLLER_STATEREF_COMPUTATION "ComAdmittanceController: stateRef computation          "
#define PROFILE_COMADMITTANCECONTROLLER_DCOMREF_COMPUTATION "ComAdmittanceController: dcomRef computation           "
#define PROFILE_COMADMITTANCECONTROLLER_COMREF_COMPUTATION "ComAdmittanceController: comRef computation            "

#define INPUT_SIGNALS m_KpSIN << m_zmpSIN << m_zmpDesSIN << m_ddcomDesSIN

#define INNER_SIGNALS m_stateRefSINNER

#define OUTPUT_SIGNALS m_comRefSOUT << m_dcomRefSOUT << m_ddcomRefSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef ComAdmittanceController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ComAdmittanceController, "ComAdmittanceController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
ComAdmittanceController::ComAdmittanceController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(zmp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(zmpDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(ddcomDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(ddcomRef, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(stateRef, dynamicgraph::Vector, m_ddcomRefSOUT),
      CONSTRUCT_SIGNAL_OUT(comRef, dynamicgraph::Vector, m_stateRefSINNER),
      CONSTRUCT_SIGNAL_OUT(dcomRef, dynamicgraph::Vector, m_stateRefSINNER)
      // dcomRef is set to depend from comRefSOUT to ensure position is updated before velocity
      ,
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &ComAdmittanceController::init,
                                      docCommandVoid1("Initialize the entity.", "time step")));
  addCommand("setPosition", makeCommandVoid1(*this, &ComAdmittanceController::setPosition,
                                             docCommandVoid1("Set initial reference position.", "Initial position")));
  addCommand("setVelocity", makeCommandVoid1(*this, &ComAdmittanceController::setVelocity,
                                             docCommandVoid1("Set initial reference velocity.", "Initial velocity")));
  addCommand("setState", makeCommandVoid2(*this, &ComAdmittanceController::setState,
                                          docCommandVoid2("Set initial reference position and velocity.",
                                                          "Initial position", "Initial velocity")));
}

void ComAdmittanceController::init(const double& dt) {
  if (!m_KpSIN.isPlugged()) return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_ddcomDesSIN.isPlugged()) return SEND_MSG("Init failed: signal ddcomDes is not plugged", MSG_TYPE_ERROR);
  if (!m_zmpSIN.isPlugged()) return SEND_MSG("Init failed: signal zmp is not plugged", MSG_TYPE_ERROR);
  if (!m_zmpDesSIN.isPlugged()) return SEND_MSG("Init failed: signal zmpDes is not plugged", MSG_TYPE_ERROR);

  m_dt = dt;
  m_state.setZero(6);
  m_initSucceeded = true;
}

void ComAdmittanceController::setPosition(const dynamicgraph::Vector& com) { m_state.head<3>() = com; }

void ComAdmittanceController::setVelocity(const dynamicgraph::Vector& dcom) { m_state.tail<3>() = dcom; }

void ComAdmittanceController::setState(const dynamicgraph::Vector& com, const dynamicgraph::Vector& dcom) {
  setPosition(com);
  setVelocity(dcom);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(ddcomRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal ddcomRef before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  getProfiler().start(PROFILE_COMADMITTANCECONTROLLER_DDCOMREF_COMPUTATION);

  const Vector& ddcomDes = m_ddcomDesSIN(iter);
  const Vector& zmp = m_zmpSIN(iter);
  const Vector& zmpDes = m_zmpDesSIN(iter);
  const Vector& Kp = m_KpSIN(iter);

  assert(ddcomDes.size() == 3 && "Unexpected size of signal ddcomDes");
  assert(zmp.size() == 3 && "Unexpected size of signal zmp");
  assert(zmpDes.size() == 3 && "Unexpected size of signal zmpDes");
  assert(Kp.size() == 3 && "Unexpected size of signal Kp");

  s = ddcomDes + Kp.cwiseProduct(zmp - zmpDes);

  getProfiler().stop(PROFILE_COMADMITTANCECONTROLLER_DDCOMREF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(stateRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal stateRef before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_COMADMITTANCECONTROLLER_STATEREF_COMPUTATION);

  const Vector& ddcomRef = m_ddcomRefSOUT(iter);

  assert(ddcomRef.size() == 3 && "Unexpected size of signal ddcomRef");

  const Eigen::Vector3d dcomRef = m_state.tail<3>();

  m_state.head<3>() += dcomRef * m_dt + 0.5 * ddcomRef * m_dt * m_dt;
  m_state.tail<3>() += ddcomRef * m_dt;

  s = m_state;

  getProfiler().stop(PROFILE_COMADMITTANCECONTROLLER_STATEREF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(comRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dcomRef before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  getProfiler().start(PROFILE_COMADMITTANCECONTROLLER_COMREF_COMPUTATION);

  const Vector& stateRef = m_stateRefSINNER(iter);

  assert(stateRef.size() == 6 && "Unexpected size of signal stateRef");

  s = stateRef.head<3>();

  getProfiler().stop(PROFILE_COMADMITTANCECONTROLLER_COMREF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dcomRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dcomRef before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  getProfiler().start(PROFILE_COMADMITTANCECONTROLLER_DCOMREF_COMPUTATION);

  const Vector& stateRef = m_stateRefSINNER(iter);

  assert(stateRef.size() == 6 && "Unexpected size of signal stateRef");

  s = stateRef.tail<3>();

  getProfiler().stop(PROFILE_COMADMITTANCECONTROLLER_DCOMREF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void ComAdmittanceController::display(std::ostream& os) const {
  os << "ComAdmittanceController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

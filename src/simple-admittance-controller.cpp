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

#include "sot/talos_balance/simple-admittance-controller.hh"

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

// Size to be aligned                                   "-------------------------------------------------------"

#define PROFILE_SIMPLE_ADMITTANCECONTROLLER_QREF_COMPUTATION \
  "SimpleAdmittanceController: qRef computation                 "

#define PROFILE_SIMPLE_ADMITTANCECONTROLLER_DQREF_COMPUTATION \
  "SimpleAdmittanceController: dqRef computation                "

#define INPUT_SIGNALS m_KpSIN << m_stateSIN << m_tauSIN << m_tauDesSIN

#define OUTPUT_SIGNALS m_qRefSOUT << m_dqRefSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleAdmittanceController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleAdmittanceController, "SimpleAdmittanceController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleAdmittanceController::SimpleAdmittanceController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(state, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tauDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(dqRef, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(qRef, dynamicgraph::Vector, m_dqRefSOUT),
      m_useState(false),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &SimpleAdmittanceController::init,
                                      docCommandVoid2("Initialize the entity.", "time step", "Number of elements")));
  addCommand("setPosition", makeCommandVoid1(*this, &SimpleAdmittanceController::setPosition,
                                             docCommandVoid1("Set initial reference position.", "Initial position")));
  addCommand("useExternalState", makeDirectSetter(*this, &m_useState, docDirectSetter("use external state", "bool")));
  addCommand("isUsingExternalState",
             makeDirectGetter(*this, &m_useState, docDirectGetter("use external state", "bool")));
}

void SimpleAdmittanceController::init(const double& dt, const unsigned& n) {
  if (n < 1) return SEND_MSG("n must be at least 1", MSG_TYPE_ERROR);
  if (!m_KpSIN.isPlugged()) return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_tauSIN.isPlugged()) return SEND_MSG("Init failed: signal tau is not plugged", MSG_TYPE_ERROR);
  if (!m_tauDesSIN.isPlugged()) return SEND_MSG("Init failed: signal tauDes is not plugged", MSG_TYPE_ERROR);

  m_n = n;
  m_dt = dt;
  m_q.setZero(n);
  m_initSucceeded = true;
}

void SimpleAdmittanceController::setPosition(const dynamicgraph::Vector& position)

{
  m_q = position;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(dqRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dqRef before initialization!");
    return s;
  }
  if (s.size() != m_n) s.resize(m_n);

  getProfiler().start(PROFILE_SIMPLE_ADMITTANCECONTROLLER_DQREF_COMPUTATION);

  const Vector& tauDes = m_tauDesSIN(iter);
  const Vector& tau = m_tauSIN(iter);
  const Vector& Kp = m_KpSIN(iter);

  assert(tau.size() == m_n && "Unexpected size of signal tau");
  assert(tauDes.size() == m_n && "Unexpected size of signal tauDes");
  assert(Kp.size() == m_n && "Unexpected size of signal Kp");

  s = Kp.cwiseProduct(tauDes - tau);

  getProfiler().stop(PROFILE_SIMPLE_ADMITTANCECONTROLLER_DQREF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(qRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal qRef before initialization!");
    return s;
  }
  if (s.size() != m_n) s.resize(m_n);

  getProfiler().start(PROFILE_SIMPLE_ADMITTANCECONTROLLER_QREF_COMPUTATION);

  const Vector& dqRef = m_dqRefSOUT(iter);

  assert(dqRef.size() == m_n && "Unexpected size of signal dqRef");

  if (m_useState) {
    if (!m_stateSIN.isPlugged()) {
      SEND_MSG("Signal state is requested, but is not plugged", MSG_TYPE_ERROR);
      return s;
    }
    const Vector& state = m_stateSIN(iter);
    assert(state.size() == m_n && "Unexpected size of signal state");
    m_q = state;
  }

  m_q += dqRef * m_dt;

  s = m_q;

  getProfiler().stop(PROFILE_SIMPLE_ADMITTANCECONTROLLER_QREF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleAdmittanceController::display(std::ostream& os) const {
  os << "SimpleAdmittanceController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

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

#include "sot/talos_balance/joint-position-controller.hh"

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

// Size to be aligned                                      "-------------------------------------------------------"
#define PROFILE_JOINTPOSITIONCONTROLLER_DQREF_COMPUTATION "JointPositionController: dqRef computation             "

#define INPUT_SIGNALS m_KpSIN << m_stateSIN << m_qDesSIN << m_dqDesSIN

#define OUTPUT_SIGNALS m_dqRefSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef JointPositionController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(JointPositionController, "JointPositionController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
JointPositionController::JointPositionController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(state, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(qDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dqDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(dqRef, dynamicgraph::Vector, INPUT_SIGNALS),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &JointPositionController::init,
                                      docCommandVoid1("Initialize the entity.", "Control gains")));
}

void JointPositionController::init(const unsigned& n) {
  if (n < 1) return SEND_MSG("n must be at least 1", MSG_TYPE_ERROR);
  if (!m_KpSIN.isPlugged()) return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_stateSIN.isPlugged()) return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);
  if (!m_qDesSIN.isPlugged()) return SEND_MSG("Init failed: signal qDes is not plugged", MSG_TYPE_ERROR);
  if (!m_dqDesSIN.isPlugged()) return SEND_MSG("Init failed: signal dqDes is not plugged", MSG_TYPE_ERROR);

  m_n = n;
  m_initSucceeded = true;
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

  getProfiler().start(PROFILE_JOINTPOSITIONCONTROLLER_DQREF_COMPUTATION);

  const Vector& state = m_stateSIN(iter);
  const Vector& qDes = m_qDesSIN(iter);
  const Vector& dqDes = m_dqDesSIN(iter);
  const Vector& Kp = m_KpSIN(iter);

  assert(state.size() == m_n + 6 && "Unexpected size of signal state");
  assert(qDes.size() == m_n && "Unexpected size of signal qDes");
  assert(dqDes.size() == m_n && "Unexpected size of signal dqDes");
  assert(Kp.size() == m_n && "Unexpected size of signal Kp");

  s = dqDes + Kp.cwiseProduct(qDes - state.tail(m_n));

  getProfiler().stop(PROFILE_JOINTPOSITIONCONTROLLER_DQREF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void JointPositionController::display(std::ostream& os) const {
  os << "JointPositionController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

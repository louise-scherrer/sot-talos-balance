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

#include "sot/talos_balance/simple-pid.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include <dynamic-graph/all-commands.h>
#include "sot/core/stop-watch.hh"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                           "-------------------------------------------------------"
#define PROFILE_SIMPLE_PID_DX_REF_COMPUTATION "SimplePID: dx_ref computation                          "

#define INPUT_SIGNALS m_KpSIN << m_KiSIN << m_decayFactorSIN << m_xSIN << m_x_desSIN << m_dx_desSIN

#define OUTPUT_SIGNALS m_dx_refSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimplePID EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimplePID, "SimplePID");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimplePID::SimplePID(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Ki, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(decayFactor, double),
      CONSTRUCT_SIGNAL_IN(x, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(x_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dx_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(dx_ref, dynamicgraph::Vector, INPUT_SIGNALS),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &SimplePID::init,
                                      docCommandVoid2("Initialize the entity.", "time step", "number of elements")));
  addCommand("resetIntegralError",
             makeCommandVoid0(*this, &SimplePID::resetIntegralError, docCommandVoid0("Set integral error to zero.")));
}

void SimplePID::init(const double& dt, const int& N) {
  m_dt = dt;
  m_integralError.setZero(N);
  m_initSucceeded = true;
}

void SimplePID::resetIntegralError() { m_integralError.setZero(); }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(dx_ref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dx_ref before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_SIMPLE_PID_DX_REF_COMPUTATION);

  const Vector& Kp = m_KpSIN(iter);
  const Vector& Ki = m_KiSIN(iter);
  const double& decayFactor = m_decayFactorSIN(iter);

  const Vector& x = m_xSIN(iter);
  const Vector& x_des = m_x_desSIN(iter);

  const Vector& dx_des = m_dx_desSIN(iter);

  Vector x_err = x_des - x;

  Vector ddx_ref = dx_des + Kp.cwiseProduct(x_err) + Ki.cwiseProduct(m_integralError);

  // update the integrator (AFTER using its value)
  m_integralError += (x_err - decayFactor * m_integralError) * m_dt;

  s = ddx_ref;

  getProfiler().stop(PROFILE_SIMPLE_PID_DX_REF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimplePID::display(std::ostream& os) const {
  os << "SimplePID " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

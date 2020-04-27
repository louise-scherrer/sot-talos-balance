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

#include "sot/talos_balance/simple-pidd.hh"

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

// Size to be aligned                              "-------------------------------------------------------"
#define PROFILE_SIMPLE_PIDD_DDX_REF_COMPUTATION "SimplePIDD: ddx_ref computation                        "

#define INPUT_SIGNALS \
  m_KpSIN << m_KiSIN << m_KdSIN << m_decayFactorSIN << m_xSIN << m_x_desSIN << m_dxSIN << m_dx_desSIN << m_ddx_desSIN

#define OUTPUT_SIGNALS m_ddx_refSOUT << m_dx_refSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimplePIDD EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimplePIDD, "SimplePIDD");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimplePIDD::SimplePIDD(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Ki, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Kd, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(decayFactor, double),
      CONSTRUCT_SIGNAL_IN(x, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(x_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dx, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dx_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(ddx_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(ddx_ref, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(dx_ref, dynamicgraph::Vector, m_ddx_refSOUT),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &SimplePIDD::init,
                                      docCommandVoid2("Initialize the entity.", "time step", "number of elements")));
  addCommand("resetIntegralError",
             makeCommandVoid0(*this, &SimplePIDD::resetIntegralError, docCommandVoid0("Set integral error to zero.")));
  addCommand("resetVelocity", makeCommandVoid0(*this, &SimplePIDD::resetIntegralError,
                                               docCommandVoid0("Set reference velocity to zero.")));
}

void SimplePIDD::init(const double& dt, const int& N) {
  m_dt = dt;
  m_integralError.setZero(N);
  m_dx_ref.setZero(N);
  m_initSucceeded = true;
}

void SimplePIDD::resetVelocity() { m_dx_ref.setZero(); }

void SimplePIDD::resetIntegralError() { m_integralError.setZero(); }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(ddx_ref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal ddx_ref before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_SIMPLE_PIDD_DDX_REF_COMPUTATION);

  const Vector& Kp = m_KpSIN(iter);
  const Vector& Ki = m_KiSIN(iter);
  const Vector& Kd = m_KiSIN(iter);
  const double& decayFactor = m_decayFactorSIN(iter);

  const Vector& x = m_xSIN(iter);
  const Vector& x_des = m_x_desSIN(iter);

  const Vector& dx = m_dxSIN(iter);
  const Vector& dx_des = m_dx_desSIN(iter);

  const Vector& ddx_des = m_ddx_desSIN(iter);

  Vector x_err = x_des - x;
  Vector dx_err = dx_des - dx;

  Vector ddx_ref = ddx_des + Kd.cwiseProduct(dx_err) + Kp.cwiseProduct(x_err) + Ki.cwiseProduct(m_integralError);

  // update the integrator (AFTER using its value)
  m_integralError += (x_err - decayFactor * m_integralError) * m_dt;

  s = ddx_ref;

  getProfiler().stop(PROFILE_SIMPLE_PIDD_DDX_REF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dx_ref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dx_ref before initialization!");
    return s;
  }

  const Vector& ddx_ref = m_ddx_refSOUT(iter);

  m_dx_ref += ddx_ref * m_dt;

  s = m_dx_ref;

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimplePIDD::display(std::ostream& os) const {
  os << "SimplePIDD " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

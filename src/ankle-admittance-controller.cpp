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

#include "sot/talos_balance/ankle-admittance-controller.hh"

#include <sot/core/debug.hh>
#include <sot/core/stop-watch.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/all-commands.h>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

#define PROFILE_ANKLEADMITTANCECONTROLLER_DRP_COMPUTATION "AnkleAdmittanceController: dRP computation                "

#define INPUT_SIGNALS m_gainsXYSIN << m_wrenchSIN << m_pRefSIN

#define OUTPUT_SIGNALS m_dRPSOUT << m_vDesSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef AnkleAdmittanceController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AnkleAdmittanceController, "AnkleAdmittanceController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
AnkleAdmittanceController::AnkleAdmittanceController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(gainsXY, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(wrench, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(pRef, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(dRP, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(vDes, dynamicgraph::Vector, m_dRPSOUT),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init",
             makeCommandVoid0(*this, &AnkleAdmittanceController::init, docCommandVoid0("Initialize the entity.")));
}

void AnkleAdmittanceController::init() {
  ;
  if (!m_gainsXYSIN.isPlugged()) return SEND_MSG("Init failed: signal gainsXY is not plugged", MSG_TYPE_ERROR);
  if (!m_wrenchSIN.isPlugged()) return SEND_MSG("Init failed: signal wrench is not plugged", MSG_TYPE_ERROR);
  if (!m_pRefSIN.isPlugged()) return SEND_MSG("Init failed: signal pRef is not plugged", MSG_TYPE_ERROR);

  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(dRP, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Can't compute dqRef before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);

  getProfiler().start(PROFILE_ANKLEADMITTANCECONTROLLER_DRP_COMPUTATION);

  const Eigen::Vector3d pRef = m_pRefSIN(iter);
  const Vector& wrench = m_wrenchSIN(iter);
  const Vector& gainsXY = m_gainsXYSIN(iter);

  assert(pRef.size() == 3 && "Unexpected size of signal pRef");
  assert(wrench.size() == 6 && "Unexpected size of signal wrench");
  assert(gainsXY.size() == 2 && "Unexpected size of signal gainsXY");

  const Eigen::Vector3d error = wrench.tail<3>() - pRef.cross(wrench.head<3>());
  Eigen::Vector2d dRP;

  dRP[0] = gainsXY[0] * error[0];
  dRP[1] = gainsXY[1] * error[1];

  s = dRP;

  getProfiler().stop(PROFILE_ANKLEADMITTANCECONTROLLER_DRP_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(vDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Can't compute vDes before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  const Vector& dRP = m_dRPSOUT(iter);

  s.head<3>().setZero();
  s.segment<2>(3) = dRP;
  s[5] = 0;

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void AnkleAdmittanceController::display(std::ostream& os) const {
  os << "AnkleAdmittanceController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

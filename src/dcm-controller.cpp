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

#include "sot/talos_balance/dcm-controller.hh"

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

// Size to be aligned                                "-------------------------------------------------------"
#define PROFILE_DCMCONTROLLER_ZMPREF_COMPUTATION "DcmController: zmpRef computation                      "
#define PROFILE_DCMCONTROLLER_WRENCHREF_COMPUTATION "DcmController: wrenchRef computation                   "

#define INPUT_SIGNALS                                                                                                 \
  m_KpSIN << m_KiSIN << m_KzSIN << m_decayFactorSIN << m_omegaSIN << m_massSIN << m_comSIN << m_dcmSIN << m_dcmDesSIN \
          << m_zmpDesSIN << m_zmpSIN

#define OUTPUT_SIGNALS m_zmpRefSOUT << m_wrenchRefSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DcmController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DcmController, "DcmController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
DcmController::DcmController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Ki, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Kz, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(decayFactor, double),
      CONSTRUCT_SIGNAL_IN(omega, double),
      CONSTRUCT_SIGNAL_IN(mass, double),
      CONSTRUCT_SIGNAL_IN(com, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dcm, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dcmDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(zmpDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(zmp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(zmpRef, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector, m_zmpRefSOUT << m_comSIN << m_omegaSIN << m_massSIN)
      // dcomRef is set to depend from comRefSOUT to ensure position is updated before velocity
      ,
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init",
             makeCommandVoid1(*this, &DcmController::init, docCommandVoid1("Initialize the entity.", "time step")));
  addCommand("resetDcmIntegralError", makeCommandVoid0(*this, &DcmController::resetDcmIntegralError,
                                                       docCommandVoid0("Set dcm integral error to zero.")));
}

void DcmController::init(const double& dt) {
  if (!m_KpSIN.isPlugged()) return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_KiSIN.isPlugged()) return SEND_MSG("Init failed: signal Ki is not plugged", MSG_TYPE_ERROR);
  if (!m_decayFactorSIN.isPlugged()) return SEND_MSG("Init failed: signal decayFactor is not plugged", MSG_TYPE_ERROR);
  if (!m_omegaSIN.isPlugged()) return SEND_MSG("Init failed: signal omega is not plugged", MSG_TYPE_ERROR);
  if (!m_massSIN.isPlugged()) return SEND_MSG("Init failed: signal mass is not plugged", MSG_TYPE_ERROR);
  if (!m_comSIN.isPlugged()) return SEND_MSG("Init failed: signal com is not plugged", MSG_TYPE_ERROR);
  if (!m_dcmSIN.isPlugged()) return SEND_MSG("Init failed: signal dcm is not plugged", MSG_TYPE_ERROR);
  if (!m_dcmDesSIN.isPlugged()) return SEND_MSG("Init failed: signal dcmDes is not plugged", MSG_TYPE_ERROR);
  if (!m_zmpDesSIN.isPlugged()) return SEND_MSG("Init failed: signal zmpDes is not plugged", MSG_TYPE_ERROR);

  m_dt = dt;
  resetDcmIntegralError();
  m_initSucceeded = true;
}

void DcmController::resetDcmIntegralError() { m_dcmIntegralError.setZero(3); }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(zmpRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmpRef before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  getProfiler().start(PROFILE_DCMCONTROLLER_ZMPREF_COMPUTATION);

  const Vector& Kp = m_KpSIN(iter);
  const Vector& Ki = m_KiSIN(iter);
  const double& decayFactor = m_decayFactorSIN(iter);
  const double& omega = m_omegaSIN(iter);
  const Vector& dcm = m_dcmSIN(iter);
  const Vector& dcmDes = m_dcmDesSIN(iter);
  const Vector& zmpDes = m_zmpDesSIN(iter);

  assert(Kp.size() == 3 && "Unexpected size of signal Kp");
  assert(Ki.size() == 3 && "Unexpected size of signal Ki");
  assert(dcm.size() == 3 && "Unexpected size of signal dcm");
  assert(dcmDes.size() == 3 && "Unexpected size of signal dcmDes");
  assert(zmpDes.size() == 3 && "Unexpected size of signal zmpDes");

  const Eigen::Vector3d dcmError = dcmDes - dcm;

  Eigen::Vector3d zmpRef = zmpDes - (Eigen::Vector3d::Constant(1.0) + Kp / omega).cwiseProduct(dcmError) -
                           Ki.cwiseProduct(m_dcmIntegralError) / omega;
  if (m_zmpSIN.isPlugged()) {
    const Vector& zmp = m_zmpSIN(iter);
    const Vector& Kz = m_KzSIN(iter);
    assert(Kz.size() == 3 && "Unexpected size of signal Kz");
    assert(zmp.size() == 3 && "Unexpected size of signal zmp");
    zmpRef += (Kz / omega) * (zmpDes - zmp);
  }
  zmpRef[2] = 0.0;  // maybe needs better way

  // update the integrator (AFTER using its value)
  m_dcmIntegralError += (dcmError - decayFactor * m_dcmIntegralError) * m_dt;

  s = zmpRef;

  getProfiler().stop(PROFILE_DCMCONTROLLER_ZMPREF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(wrenchRef, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchRef before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_DCMCONTROLLER_WRENCHREF_COMPUTATION);

  const double& omega = m_omegaSIN(iter);
  const double& mass = m_massSIN(iter);
  const Vector& com = m_comSIN(iter);

  const Vector& zmpRef = m_zmpRefSOUT(iter);

  assert(com.size() == 3 && "Unexpected size of signal com");

  Eigen::Vector3d forceRef = mass * omega * omega * (com - zmpRef);
  forceRef[2] = mass * 9.81;  // maybe needs better way

  Eigen::Matrix<double, 6, 1> wrenchRef;
  wrenchRef.head<3>() = forceRef;
  const Eigen::Vector3d com3 = com;
  wrenchRef.tail<3>() = com3.cross(forceRef);

  s = wrenchRef;

  getProfiler().stop(PROFILE_DCMCONTROLLER_WRENCHREF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void DcmController::display(std::ostream& os) const {
  os << "DcmController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

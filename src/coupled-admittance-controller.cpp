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

#include "sot/talos_balance/coupled-admittance-controller.hh"

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

#define PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUSUM_COMPUTATION \
  "CoupledAdmittanceController: tauSum computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDIFF_COMPUTATION \
  "CoupledAdmittanceController: tauDiff computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDESSUM_COMPUTATION \
  "CoupledAdmittanceController: tauDesSum computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDESDIFF_COMPUTATION \
  "CoupledAdmittanceController: tauDesDiff computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFSUM_COMPUTATION \
  "CoupledAdmittanceController: dqRefSum computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFDIFF_COMPUTATION \
  "CoupledAdmittanceController: dqReDiff computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFL_COMPUTATION \
  "CoupledAdmittanceController: dqRefL computation                "
#define PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFR_COMPUTATION \
  "CoupledAdmittanceController: dqRefR computation                "

#define INPUT_SIGNALS m_kSumSIN << m_kDiffSIN << m_tauLSIN << m_tauRSIN << m_tauDesLSIN << m_tauDesRSIN

#define INNER_SIGNALS \
  m_tauSumSINNER << m_tauDiffSINNER << m_tauDesSumSINNER << m_tauDesDiffSINNER << m_dqRefSumSINNER << m_dqRefDiffSINNER

#define OUTPUT_SIGNALS m_dqRefLSOUT << m_dqRefRSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef CoupledAdmittanceController EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CoupledAdmittanceController, "CoupledAdmittanceController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
CoupledAdmittanceController::CoupledAdmittanceController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(kSum, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kDiff, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tauL, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tauR, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tauDesL, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tauDesR, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_INNER(tauSum, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(tauDiff, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(tauDesSum, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(tauDesDiff, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_INNER(dqRefSum, dynamicgraph::Vector, INPUT_SIGNALS << m_tauSumSINNER << m_tauDesSumSINNER),
      CONSTRUCT_SIGNAL_INNER(dqRefDiff, dynamicgraph::Vector, INPUT_SIGNALS << m_tauDiffSINNER << m_tauDesDiffSINNER),
      CONSTRUCT_SIGNAL_OUT(dqRefL, dynamicgraph::Vector, INNER_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(dqRefR, dynamicgraph::Vector, INNER_SIGNALS) {
  Entity::signalRegistration(INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */
DEFINE_SIGNAL_INNER_FUNCTION(tauSum, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUSUM_COMPUTATION);

  const Vector& tauL = m_tauLSIN(iter);
  const Vector& tauR = m_tauRSIN(iter);

  s = tauL + tauR;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUSUM_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(tauDiff, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDIFF_COMPUTATION);

  const Vector& tauL = m_tauLSIN(iter);
  const Vector& tauR = m_tauRSIN(iter);

  s = tauL - tauR;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDIFF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(tauDesSum, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDESSUM_COMPUTATION);

  const Vector& tauDesL = m_tauDesLSIN(iter);
  const Vector& tauDesR = m_tauDesRSIN(iter);

  s = tauDesL + tauDesR;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDESSUM_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(tauDesDiff, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDESDIFF_COMPUTATION);

  const Vector& tauDesL = m_tauDesLSIN(iter);
  const Vector& tauDesR = m_tauDesRSIN(iter);

  s = tauDesL - tauDesR;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_TAUDIFF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(dqRefSum, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFSUM_COMPUTATION);

  const Vector& tau = m_tauSumSINNER(iter);
  const Vector& tauDes = m_tauDesSumSINNER(iter);
  const Vector& k = m_kSumSIN(iter);

  s = k.cwiseProduct(tauDes - tau);

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFSUM_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(dqRefDiff, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFDIFF_COMPUTATION);

  const Vector& tau = m_tauDiffSINNER(iter);
  const Vector& tauDes = m_tauDesDiffSINNER(iter);
  const Vector& k = m_kDiffSIN(iter);

  s = k.cwiseProduct(tauDes - tau);

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFDIFF_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dqRefL, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFL_COMPUTATION);

  const Vector& dqRefSum = m_dqRefSumSINNER(iter);
  const Vector& dqRefDiff = m_dqRefDiffSINNER(iter);

  s = dqRefSum + dqRefDiff;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFL_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dqRefR, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFR_COMPUTATION);

  const Vector& dqRefSum = m_dqRefSumSINNER(iter);
  const Vector& dqRefDiff = m_dqRefDiffSINNER(iter);

  s = dqRefSum - dqRefDiff;

  getProfiler().stop(PROFILE_COUPLED_ADMITTANCECONTROLLER_DQREFR_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void CoupledAdmittanceController::display(std::ostream& os) const {
  os << "CoupledAdmittanceController " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

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

#include "sot/talos_balance/dummy-dcm-estimator.hh"

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

// Size to be aligned                              "-------------------------------------------------------"
#define PROFILE_DUMMYDCMESTIMATOR_DCM_COMPUTATION "DummyDcmEstimator: dcm computation                     "

#define INPUT_SIGNALS m_omegaSIN << m_massSIN << m_comSIN << m_momentaSIN

#define OUTPUT_SIGNALS m_dcmSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DummyDcmEstimator EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DummyDcmEstimator, "DummyDcmEstimator");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
DummyDcmEstimator::DummyDcmEstimator(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(omega, double),
      CONSTRUCT_SIGNAL_IN(mass, double),
      CONSTRUCT_SIGNAL_IN(com, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(momenta, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(dcm, dynamicgraph::Vector, INPUT_SIGNALS),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &DummyDcmEstimator::init, docCommandVoid0("Initialize the entity.")));
}

void DummyDcmEstimator::init() {
  if (!m_omegaSIN.isPlugged()) return SEND_MSG("Init failed: signal omega is not plugged", MSG_TYPE_ERROR);
  if (!m_massSIN.isPlugged()) return SEND_MSG("Init failed: signal mass is not plugged", MSG_TYPE_ERROR);
  if (!m_comSIN.isPlugged()) return SEND_MSG("Init failed: signal com is not plugged", MSG_TYPE_ERROR);
  if (!m_momentaSIN.isPlugged()) return SEND_MSG("Init failed: signal momenta is not plugged", MSG_TYPE_ERROR);

  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(dcm, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal com_dcom before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  getProfiler().start(PROFILE_DUMMYDCMESTIMATOR_DCM_COMPUTATION);

  const double& omega = m_omegaSIN(iter);
  const double& mass = m_massSIN(iter);
  const Vector& com = m_comSIN(iter);
  const Vector& momenta = m_momentaSIN(iter);

  assert(com.size() == 3 && "Unexpected size of signal com");
  assert((momenta.size() == 3 || momenta.size() == 6) && "Unexpected size of signal momenta");

  const Eigen::Vector3d dcom = momenta.head<3>() / mass;

  const Eigen::Vector3d dcm = com + dcom / omega;

  s = dcm;

  getProfiler().stop(PROFILE_DUMMYDCMESTIMATOR_DCM_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void DummyDcmEstimator::display(std::ostream& os) const {
  os << "DummyDcmEstimator " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

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

#include "sot/talos_balance/state-transformation.hh"

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

// Size to be aligned                               "-------------------------------------------------------"
#define PROFILE_STATETRANSFORMATION_Q_COMPUTATION "State transformation q computation                     "
#define PROFILE_STATETRANSFORMATION_V_COMPUTATION "State transformation v computation                     "

#define INPUT_SIGNALS m_referenceFrameSIN << m_q_inSIN << m_v_inSIN

#define OUTPUT_SIGNALS m_qSOUT << m_vSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef StateTransformation EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StateTransformation, "StateTransformation");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
StateTransformation::StateTransformation(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(referenceFrame, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(q_in, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(v_in, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(q, dynamicgraph::Vector, m_referenceFrameSIN << m_q_inSIN),
      CONSTRUCT_SIGNAL_OUT(v, dynamicgraph::Vector, m_referenceFrameSIN << m_v_inSIN),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &StateTransformation::init, docCommandVoid0("Initialize the entity.")));
}

void StateTransformation::init() { m_initSucceeded = true; }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(q, dynamicgraph::Vector) {
  const dynamicgraph::Vector& input = m_q_inSIN(iter);
  const size_t sz = input.size();
  if ((size_t)(s.size()) != sz) s.resize(sz);

  getProfiler().start(PROFILE_STATETRANSFORMATION_Q_COMPUTATION);

  const MatrixHomogeneous& referenceFrame = m_referenceFrameSIN(iter);

  // convert q base to homogeneous matrix
  const Eigen::Vector3d& euler = input.segment<3>(3);

  const double roll = euler[0];
  const double pitch = euler[1];
  const double yaw = euler[2];

  Eigen::Quaterniond quat;
  quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  MatrixHomogeneous M;
  M.linear() = quat.toRotationMatrix();
  M.translation() = input.head<3>();

  // Convert the matrix
  MatrixHomogeneous res = referenceFrame * M;

  // Write the result
  s.head<3>() = res.translation();

  s.segment<3>(3) = res.linear().eulerAngles(2, 1, 0).reverse();

  if (sz > 6) s.tail(sz - 6) = input.tail(sz - 6);

  getProfiler().stop(PROFILE_STATETRANSFORMATION_Q_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(v, dynamicgraph::Vector) {
  const dynamicgraph::Vector& input = m_v_inSIN(iter);
  const size_t sz = input.size();
  if ((size_t)(s.size()) != sz) s.resize(sz);

  getProfiler().start(PROFILE_STATETRANSFORMATION_V_COMPUTATION);

  const MatrixHomogeneous& referenceFrame = m_referenceFrameSIN(iter);

  // Write the result
  s.head<3>() = referenceFrame.linear() * input.head<3>();

  s.segment<3>(3) = referenceFrame.linear() * input.segment<3>(3);

  if (sz > 6) s.tail(sz - 6) = input.tail(sz - 6);

  getProfiler().stop(PROFILE_STATETRANSFORMATION_V_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void StateTransformation::display(std::ostream& os) const {
  os << "StateTransformation " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

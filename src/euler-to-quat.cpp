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

#include "sot/talos_balance/euler-to-quat.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

#include <Eigen/Core>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                       "-------------------------------------------------------"
#define PROFILE_EULERTOQUAT_COMPUTATION "EulerToQuat computation                                  "

#define INPUT_SIGNALS m_eulerSIN

#define OUTPUT_SIGNALS m_quaternionSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef EulerToQuat EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(EulerToQuat, "EulerToQuat");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
EulerToQuat::EulerToQuat(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(euler, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(quaternion, dynamicgraph::Vector, INPUT_SIGNALS) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &EulerToQuat::init, docCommandVoid0("Initialize the entity.")));
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(quaternion, dynamicgraph::Vector) {
  const dynamicgraph::Vector& input = m_eulerSIN(iter);
  const size_t sz = input.size();
  if ((size_t)(s.size()) != (sz + 1)) s.resize(sz + 1);

  getProfiler().start(PROFILE_EULERTOQUAT_COMPUTATION);

  const Eigen::Vector3d& euler = input.segment<3>(3);

  const double roll = euler[0];
  const double pitch = euler[1];
  const double yaw = euler[2];

  Eigen::Quaterniond quat;
  quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  s.head<3>() = input.head<3>();

  s.segment<4>(3) = quat.coeffs();

  if (sz > 6) s.tail(sz - 6) = input.tail(sz - 6);

  getProfiler().stop(PROFILE_EULERTOQUAT_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void EulerToQuat::display(std::ostream& os) const {
  os << "EulerToQuat " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

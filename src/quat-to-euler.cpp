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

#include "sot/talos_balance/quat-to-euler.hh"

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
#define PROFILE_QUATTOEULER_COMPUTATION "QuatToEuler computation                                "

#define INPUT_SIGNALS m_quaternionSIN

#define OUTPUT_SIGNALS m_eulerSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef QuatToEuler EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(QuatToEuler, "QuatToEuler");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
QuatToEuler::QuatToEuler(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(quaternion, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(euler, dynamicgraph::Vector, INPUT_SIGNALS) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &QuatToEuler::init, docCommandVoid0("Initialize the entity.")));
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(euler, dynamicgraph::Vector) {
  const dynamicgraph::Vector& input = m_quaternionSIN(iter);
  const size_t sz = input.size();
  if ((size_t)(s.size()) != (sz - 1)) s.resize(sz - 1);

  getProfiler().start(PROFILE_QUATTOEULER_COMPUTATION);

  const Eigen::Map<const Eigen::Quaterniond> quat(input.segment<4>(3).data());

  s.head<3>() = input.head<3>();

  Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
  s[3] = euler[2];
  s[4] = euler[1];
  s[5] = euler[0];

  if (sz > 7) s.tail(sz - 7) = input.tail(sz - 7);

  getProfiler().stop(PROFILE_QUATTOEULER_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void QuatToEuler::display(std::ostream& os) const {
  os << "QuatToEuler " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

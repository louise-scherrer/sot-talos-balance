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

#include "sot/talos_balance/simple-controller-6d.hh"

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

// Size to be aligned                                     "-------------------------------------------------------"
#define PROFILE_SIMPLE_CONTROLLER_6D_DX_REF_COMPUTATION "SimpleController6d: v_ref computation                 "

#define INPUT_SIGNALS m_KpSIN << m_xSIN << m_x_desSIN << m_v_desSIN

#define OUTPUT_SIGNALS m_v_refSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleController6d EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleController6d, "SimpleController6d");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleController6d::SimpleController6d(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(x, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(x_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(v_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(v_ref, dynamicgraph::Vector, INPUT_SIGNALS),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &SimpleController6d::init, docCommandVoid0("Initialize the entity.")));
}

void SimpleController6d::init() { m_initSucceeded = true; }

template <typename Derived>
Eigen::Matrix3d SimpleController6d::skew(const Eigen::MatrixBase<Derived>& v) {
  Eigen::Matrix3d M;
  M << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return M;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(v_ref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal v_ref before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_SIMPLE_CONTROLLER_6D_DX_REF_COMPUTATION);

  const Vector& Kp = m_KpSIN(iter);

  const MatrixHomogeneous& x = m_xSIN(iter);
  const MatrixHomogeneous& x_des = m_x_desSIN(iter);

  // const MatrixHomogeneous & x_err = x_des * x.inverse();

  const Eigen::Vector3d e_O =
      0.5 * (x.linear().col(0).cross(x_des.linear().col(0)) + x.linear().col(1).cross(x_des.linear().col(1)) +
             x.linear().col(2).cross(x_des.linear().col(2)));

  const Eigen::Matrix3d L = -0.5 * (skew(x_des.linear().col(0)) * skew(x.linear().col(0)) +
                                    skew(x_des.linear().col(1)) * skew(x.linear().col(1)) +
                                    skew(x_des.linear().col(2)) * skew(x.linear().col(2)));

  Eigen::Matrix<double, 6, 1> dv_ref;

  // dv_ref.head<3>() = Kp.head<3>().cwiseProduct(x_err.translation());
  dv_ref.head<3>() = x.linear().transpose() * Kp.head<3>().cwiseProduct(x_des.translation() - x.translation());

  dv_ref.tail<3>() = x.linear().transpose() * L.inverse() * Kp.tail<3>().cwiseProduct(e_O);

  if (m_v_desSIN.isPlugged()) {
    const dynamicgraph::Vector& v_des = m_v_desSIN(iter);
    s = v_des + dv_ref;
  } else {
    s = dv_ref;
  }

  getProfiler().stop(PROFILE_SIMPLE_CONTROLLER_6D_DX_REF_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleController6d::display(std::ostream& os) const {
  os << "SimpleController6d " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

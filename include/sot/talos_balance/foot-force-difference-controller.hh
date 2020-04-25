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

#ifndef __sot_talos_balance_foot_force_difference_controller_H__
#define __sot_talos_balance_foot_force_difference_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(foot_force_difference_controller_EXPORTS)
#define FOOT_FORCE_DIFFERENCE_CONTROLLER_EXPORT __declspec(dllexport)
#else
#define FOOT_FORCE_DIFFERENCE_CONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define FOOT_FORCE_DIFFERENCE_CONTROLLER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

#include <sot/core/matrix-geometry.hh>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class FOOT_FORCE_DIFFERENCE_CONTROLLER_EXPORT FootForceDifferenceController : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  FootForceDifferenceController(const std::string& name);

  void init();

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(phase, int);

  DECLARE_SIGNAL_IN(gainSwing, double);
  DECLARE_SIGNAL_IN(gainStance, double);
  DECLARE_SIGNAL_IN(gainDouble, double);

  DECLARE_SIGNAL_IN(dfzAdmittance, double);
  DECLARE_SIGNAL_IN(vdcFrequency, double);
  DECLARE_SIGNAL_IN(vdcDamping, double);

  DECLARE_SIGNAL_IN(swingAdmittance, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(wrenchRightDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrenchLeftDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrenchRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrenchLeft, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(posRightDes, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(posLeftDes, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(posRight, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(posLeft, MatrixHomogeneous);

  DECLARE_SIGNAL_INNER(dz_ctrl, double);
  DECLARE_SIGNAL_INNER(dz_pos, double);

  DECLARE_SIGNAL_OUT(vRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(vLeft, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(gainRight, double);
  DECLARE_SIGNAL_OUT(gainLeft, double);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  Eigen::Vector3d calcSwingAdmittance(const dynamicgraph::Vector& wrench, const dynamicgraph::Vector& swingAdmittance);

  double m_eps;
  bool m_initSucceeded;  /// true if the entity has been successfully initialized

};  // class FootForceDifferenceController

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_foot_force_difference_controller_H__

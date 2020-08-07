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

#ifndef __sot_talos_balance_ankle_admittance_controller_H__
#define __sot_talos_balance_ankle_admittance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(ankle_admittance_controller_EXPORTS)
#define ANKLEADMITTANCECONTROLLER_EXPORT __declspec(dllexport)
#else
#define ANKLEADMITTANCECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define ANKLEADMITTANCECONTROLLER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

// why these two
#include <sot/core/matrix-geometry.hh>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ANKLEADMITTANCECONTROLLER_EXPORT AnkleAdmittanceController : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  AnkleAdmittanceController(const std::string& name);

  void init(const double& dt); // new
  void setState(const Eigen::Matrix4d &); //new

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(gainsXY, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrench, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(pRef, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(footRef, MatrixHomogeneous); //new

  DECLARE_SIGNAL_OUT(dRP, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(vDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(poseDes, MatrixHomogeneous); //new

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  double m_dt; //new
  MatrixHomogeneous m_footDes; //new

};  // class AnkleAdmittanceController

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_ankle_admittance_controller_H__

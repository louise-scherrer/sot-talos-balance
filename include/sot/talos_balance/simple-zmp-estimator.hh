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

#ifndef __sot_talos_balance_simple_zmp_estimator_H__
#define __sot_talos_balance_simple_zmp_estimator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(simple_zmp_estimator_EXPORTS)
#define SIMPLEZMPESTIMATOR_EXPORT __declspec(dllexport)
#else
#define SIMPLEZMPESTIMATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SIMPLEZMPESTIMATOR_EXPORT
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

class SIMPLEZMPESTIMATOR_EXPORT SimpleZmpEstimator : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleZmpEstimator(const std::string& name, const double& eps = 1.0);

  void init();

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(wrenchLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrenchRight, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(poseLeft, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(poseRight, MatrixHomogeneous);

  DECLARE_SIGNAL_OUT(copLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(copRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(emergencyStop, bool);

  double m_eps;

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  Eigen::Vector3d computeCoP(const dynamicgraph::Vector& wrench, const MatrixHomogeneous& pose) const;

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  bool m_emergency_stop_triggered;

};  // class SimpleZmpEstimator

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_simple_zmp_estimator_H__

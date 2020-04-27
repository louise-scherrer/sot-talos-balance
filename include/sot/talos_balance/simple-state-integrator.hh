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

#ifndef __sot_talos_balance_simple_state_integrator_H__
#define __sot_talos_balance_simple_state_integrator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(simple_state_integrator_EXPORTS)
#define SIMPLE_STATE_INTEGRATOR_EXPORT __declspec(dllexport)
#else
#define SIMPLE_STATE_INTEGRATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SIMPLE_STATE_INTEGRATOR_EXPORT
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

class SIMPLE_STATE_INTEGRATOR_EXPORT SimpleStateIntegrator : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 protected:
  /// \brief Current integration step.
  double timestep_;

  /// \name Vectors related to the state.
  ///@{
  /// Position of the robot wrt pinocchio.
  Eigen::VectorXd state_;
  /// Velocity of the robot wrt pinocchio.
  Eigen::VectorXd velocity_;
  ///@}

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleStateIntegrator(const std::string& name);

  void init(const double& step);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(control, ::dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(state, ::dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(velocity, ::dynamicgraph::Vector);

 public:
  void setState(const ::dynamicgraph::Vector& st);
  void setVelocity(const ::dynamicgraph::Vector& vel);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  /// Integrate the freeflyer state (to obtain position).
  /// Compute roll pitch yaw angles
  void integrateRollPitchYaw(::dynamicgraph::Vector& state, const ::dynamicgraph::Vector& control, double dt);
  // Computes Euler angles in good range : [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]
  void rotationMatrixToEuler(const Eigen::Matrix3d& rotationMatrix, Eigen::Vector3d& rollPitchYaw);

};  // class SimpleStateIntegrator

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_simple_state_integrator_H__

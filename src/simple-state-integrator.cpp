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

#include "sot/talos_balance/simple-state-integrator.hh"

#include <cmath>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

#include <Eigen/Core>

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
//#include "pinocchio/math/quaternion.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                       "-------------------------------------------------------"
#define PROFILE_SIMPLE_STATE_INTEGRATOR_COMPUTATION \
  "SimpleStateIntegrator computation                                  "

#define INPUT_SIGNALS m_controlSIN

#define OUTPUT_SIGNALS m_stateSOUT << m_velocitySOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleStateIntegrator EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleStateIntegrator, "SimpleStateIntegrator");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleStateIntegrator::SimpleStateIntegrator(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(control, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(state, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(velocity, dynamicgraph::Vector, NULL) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */

  std::string docstring;

  docstring =
      "\n"
      "    Set integration timestep value\n"
      "\n";
  addCommand("init",
             new command::Setter<SimpleStateIntegrator, double>(*this, &SimpleStateIntegrator::init, docstring));

  docstring =
      "\n"
      "    Set state vector value\n"
      "\n";
  addCommand("setState",
             new command::Setter<SimpleStateIntegrator, Vector>(*this, &SimpleStateIntegrator::setState, docstring));

  docstring =
      "\n"
      "    Set velocity vector value\n"
      "\n";
  addCommand("setVelocity", new command::Setter<SimpleStateIntegrator, Vector>(
                                *this, &SimpleStateIntegrator::setVelocity, docstring));
}

void SimpleStateIntegrator::init(const double& step) { timestep_ = step; }

void SimpleStateIntegrator::setState(const dg::Vector& st) { state_ = st; }

void SimpleStateIntegrator::setVelocity(const dg::Vector& vel) { velocity_ = vel; }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleStateIntegrator::integrateRollPitchYaw(Vector& state, const Vector& control, double dt) {
  using Eigen::AngleAxisd;
  using Eigen::Matrix3d;
  using Eigen::QuaternionMapd;
  using Eigen::Vector3d;

  typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = state.head<3>();

  QuaternionMapd quat(qin.tail<4>().data());
  quat = AngleAxisd(state(5), Vector3d::UnitZ()) * AngleAxisd(state(4), Vector3d::UnitY()) *
         AngleAxisd(state(3), Vector3d::UnitX());

  SE3().integrate(qin, control.head<6>() * dt, qout);

  Matrix3d rotationMatrix = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix();
  // Create the Euler angles in good range : [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]
  Vector3d rollPitchYaw;
  rotationMatrixToEuler(rotationMatrix, rollPitchYaw);
  // Update freeflyer state (pose)
  state.head<3>() = qout.head<3>();
  state.segment<3>(3) << rollPitchYaw;
}

void SimpleStateIntegrator::rotationMatrixToEuler(const Eigen::Matrix3d& rotationMatrix,
                                                  Eigen::Vector3d& rollPitchYaw) {
  double m = sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2));
  double p = atan2(-rotationMatrix(2, 0), m);
  double r, y;
  if (fabs(fabs(p) - M_PI / 2.) < 0.001) {
    r = 0;
    y = -atan2(rotationMatrix(0, 1), rotationMatrix(1, 1));
  } else {
    y = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));  // alpha
    r = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));  // gamma
  }
  rollPitchYaw << r, p, y;
}

DEFINE_SIGNAL_OUT_FUNCTION(state, dynamicgraph::Vector) {
  m_velocitySOUT.setConstant(velocity_);
  m_velocitySOUT.setTime(iter + 1);

  const dynamicgraph::Vector& control = m_controlSIN(iter);

  const size_t sz = control.size();
  if ((size_t)(s.size()) != sz) s.resize(sz);
  if ((size_t)(state_.size()) != sz) throw std::runtime_error("Mismatching state and control size");
  if ((size_t)(velocity_.size()) != sz) throw std::runtime_error("Mismatching velocity and control size");

  velocity_ = control;

  integrateRollPitchYaw(state_, control, timestep_);
  state_.tail(sz - 6) += control.tail(sz - 6) * timestep_;

  s = state_;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(velocity, dynamicgraph::Vector) {
  s = velocity_;
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleStateIntegrator::display(std::ostream& os) const {
  os << "SimpleStateIntegrator " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

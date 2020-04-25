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

#ifndef __sot_talos_balance_hip_flexibility_compensation_H__
#define __sot_talos_balance_hip_flexibility_compensation_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(hip_flexibility_compensation_EXPORTS)
#define HIPFLEXIBILITYCOMPENSATION_EXPORT __declspec(dllexport)
#else
#define HIPFLEXIBILITYCOMPENSATION_EXPORT __declspec(dllimport)
#endif
#else
#define HIPFLEXIBILITYCOMPENSATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include <sot/core/robot-utils.hh>
#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class HIPFLEXIBILITYCOMPENSATION_EXPORT HipFlexibilityCompensation : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  HipFlexibilityCompensation(const std::string& name);

  /* --- SIGNALS --- */
  /// \brief Walking phase
  DECLARE_SIGNAL_IN(phase, int);

  /// \brief  Desired joint configuration of the robot
  DECLARE_SIGNAL_IN(q_des, dynamicgraph::Vector);
  /// \brief  Current torque mesured at each joint
  DECLARE_SIGNAL_IN(tau, dynamicgraph::Vector);
  /// \brief Left flexibility correction for the angular computation
  DECLARE_SIGNAL_IN(K_l, double);
  /// \brief Right flexibility correction for the angular computation
  DECLARE_SIGNAL_IN(K_r, double);
  /// \brief Derivative gain (double) for the error
  // DECLARE_SIGNAL_IN(K_d, double);

  /// \brief  Low pass filter of the signal tau
  DECLARE_SIGNAL_OUT(tau_filt, dynamicgraph::Vector);
  /// \brief  Angular correction of the flexibility
  DECLARE_SIGNAL_OUT(delta_q, dynamicgraph::Vector);
  /// \brief  Corrected desired joint configuration of the robot with flexibility joint configuration
  /// q_cmd = q_des + RateLimiter(delta_q)
  DECLARE_SIGNAL_OUT(q_cmd, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  /// \brief Initialize the entity
  void init(const double& dt, const std::string& robotName);
  /// \brief Set the LowPassFilter frequency for the torque computation.
  void setTorqueLowPassFilterFrequency(const double& frequency);
  /// \brief Set the value of the saturation for the angular correction computation.
  void setAngularSaturation(const double& saturation);
  /// \brief Set the value of the limiter for the the rate limiter of delta_q.
  void setRateLimiter(const double& rate);
  /// \brief Compute the low pass filter of a signal given a frequency and the previous signal.
  dynamicgraph::Vector lowPassFilter(const double& frequency, const dynamicgraph::Vector& signal,
                                     dynamicgraph::Vector& previous_signal);
  /// \brief Compute the limiter of a signal given the previous signal (based on first derivative).
  void rateLimiter(const dynamicgraph::Vector& signal, dynamicgraph::Vector& previous_signal,
                   dynamicgraph::Vector& output);

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  // time step of the robot
  double m_dt;
  double m_torqueLowPassFilterFrequency;
  double m_delta_q_saturation;
  double m_rate_limiter;
  dynamicgraph::Vector m_previous_delta_q;
  dynamicgraph::Vector m_previous_tau;
  dynamicgraph::Vector m_limitedSignal;

  RobotUtilShrPtr m_robot_util;

};  // class HipFlexibilityCompensation

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_hip_flexibility_compensation_H__

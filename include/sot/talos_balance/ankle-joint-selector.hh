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

#ifndef __sot_talos_balance_ankle_joint_selector_H__
#define __sot_talos_balance_ankle_joint_selector_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(position_controller_EXPORTS)
#define ANKLE_JOINT_SELECTOR_EXPORT __declspec(dllexport)
#else
#define ANKLE_JOINT_SELECTOR_EXPORT __declspec(dllimport)
#endif
#else
#define ANKLE_JOINT_SELECTOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include "boost/assign.hpp"
#include <sot/core/flags.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ANKLE_JOINT_SELECTOR_EXPORT AnkleJointSelector : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  AnkleJointSelector(const std::string& name);

  void init(const int& n);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(phase, int);

  DECLARE_SIGNAL_IN(rightRollCoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rightRollDecoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rightPitchCoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rightPitchDecoupled, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(leftRollCoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(leftRollDecoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(leftPitchCoupled, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(leftPitchDecoupled, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(selecLeft, Flags);
  DECLARE_SIGNAL_OUT(selecRight, Flags);

  DECLARE_SIGNAL_OUT(rightRoll, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(rightPitch, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(leftRoll, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(leftPitch, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  Flags m_zeros;
  Flags m_ones;
  int m_n;
  bool m_initSucceeded;  /// true if the entity has been successfully initialized

};  // class AnkleJointSelector

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_ankle_joint_selector_H__

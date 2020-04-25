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

#ifndef __sot_talos_balance_simple_reference_frame_H__
#define __sot_talos_balance_simple_reference_frame_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(simple_reference_frame_EXPORTS)
#define SIMPLEREFERENCEFRAME_EXPORT __declspec(dllexport)
#else
#define SIMPLEREFERENCEFRAME_EXPORT __declspec(dllimport)
#endif
#else
#define SIMPLEREFERENCEFRAME_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

#include <sot/core/robot-utils.hh>
#include <sot/core/matrix-geometry.hh>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SIMPLEREFERENCEFRAME_EXPORT SimpleReferenceFrame : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleReferenceFrame(const std::string& name);

  void init(const std::string& robotName);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(footLeft, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(footRight, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(reset, bool);

  DECLARE_SIGNAL_OUT(referenceFrame, MatrixHomogeneous);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  RobotUtilShrPtr m_robot_util;
  Vector m_rightFootSoleXYZ;
  MatrixHomogeneous m_referenceFrame;
  bool m_first;
  bool m_initSucceeded;  /// true if the entity has been successfully initialized

};  // class SimpleReferenceFrame

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_simple_reference_frame_H__

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

#ifndef __sot_talos_balance_dummy_walking_pattern_generator_H__
#define __sot_talos_balance_dummy_walking_pattern_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(dummy_walking_pattern_generator_EXPORTS)
#define DUMMYWALKINGPATTERNGENERATOR_EXPORT __declspec(dllexport)
#else
#define DUMMYWALKINGPATTERNGENERATOR_EXPORT __declspec(dllimport)
#endif
#else
#define DUMMYWALKINGPATTERNGENERATOR_EXPORT
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

class DUMMYWALKINGPATTERNGENERATOR_EXPORT DummyWalkingPatternGenerator : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  DummyWalkingPatternGenerator(const std::string& name);

  void init();

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(omega, double);
  DECLARE_SIGNAL_IN(rho, double);
  DECLARE_SIGNAL_IN(phase, int);

  DECLARE_SIGNAL_IN(footLeft, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(footRight, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(waist, MatrixHomogeneous);

  DECLARE_SIGNAL_IN(com, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(vcom, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(acom, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(zmp, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(referenceFrame, MatrixHomogeneous);

  DECLARE_SIGNAL_INNER(rf, MatrixHomogeneous);

  DECLARE_SIGNAL_OUT(comDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(vcomDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(acomDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dcmDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmpDes, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(footLeftDes, MatrixHomogeneous);
  DECLARE_SIGNAL_OUT(footRightDes, MatrixHomogeneous);
  DECLARE_SIGNAL_OUT(waistDes, MatrixHomogeneous);

  DECLARE_SIGNAL_OUT(omegaDes, double);
  DECLARE_SIGNAL_OUT(rhoDes, double);
  DECLARE_SIGNAL_OUT(phaseDes, int);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized

  dynamicgraph::Vector actInv(MatrixHomogeneous m, dynamicgraph::Vector v);
  MatrixHomogeneous actInv(MatrixHomogeneous m1, MatrixHomogeneous m2);

};  // class DummyWalkingPatternGenerator

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_dummy_walking_pattern_generator_H__

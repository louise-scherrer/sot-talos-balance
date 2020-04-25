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

#ifndef __sot_talos_balance_admittance_controller_H__
#define __sot_talos_balance_admittance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(admittance_controller_EXPORTS)
#define COUPLEDADMITTANCECONTROLLER_EXPORT __declspec(dllexport)
#else
#define COUPLEDADMITTANCECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define COUPLEDADMITTANCECONTROLLER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class COUPLEDADMITTANCECONTROLLER_EXPORT CoupledAdmittanceController : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  CoupledAdmittanceController(const std::string& name);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(kSum, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kDiff, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(tauL, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tauR, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(tauDesL, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tauDesR, dynamicgraph::Vector);

  DECLARE_SIGNAL_INNER(tauSum, dynamicgraph::Vector);
  DECLARE_SIGNAL_INNER(tauDiff, dynamicgraph::Vector);

  DECLARE_SIGNAL_INNER(tauDesSum, dynamicgraph::Vector);
  DECLARE_SIGNAL_INNER(tauDesDiff, dynamicgraph::Vector);

  DECLARE_SIGNAL_INNER(dqRefSum, dynamicgraph::Vector);
  DECLARE_SIGNAL_INNER(dqRefDiff, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(dqRefL, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dqRefR, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

};  // class AdmittanceController

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_admittance_controller_H__

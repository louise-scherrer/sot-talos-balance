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
#define SIMPLEADMITTANCECONTROLLER_EXPORT __declspec(dllexport)
#else
#define SIMPLEADMITTANCECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define SIMPLEADMITTANCECONTROLLER_EXPORT
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

class SIMPLEADMITTANCECONTROLLER_EXPORT SimpleAdmittanceController : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleAdmittanceController(const std::string& name);

  void init(const double& dt, const unsigned& n);

  void setPosition(const dynamicgraph::Vector& position);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(state, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tau, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tauDes, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(dqRef, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(qRef, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  bool m_useState;

 protected:
  int m_n;
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  dynamicgraph::Vector m_Kp;
  dynamicgraph::Vector m_q;  // internal state
  double m_dt;

};  // class AdmittanceController

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_admittance_controller_H__

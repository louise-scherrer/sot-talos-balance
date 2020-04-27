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

#ifndef __sot_talos_balance_boolean_identity_H__
#define __sot_talos_balance_boolean_identity_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(position_controller_EXPORTS)
#define BOOLEAN_IDENTITY_EXPORT __declspec(dllexport)
#else
#define BOOLEAN_IDENTITY_EXPORT __declspec(dllimport)
#endif
#else
#define BOOLEAN_IDENTITY_EXPORT
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

class BOOLEAN_IDENTITY_EXPORT BooleanIdentity : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  BooleanIdentity(const std::string& name);

  void init() {}

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(sin, bool);

  DECLARE_SIGNAL_OUT(sout, bool);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

};  // class BooleanIdentity

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_boolean_identity_H__

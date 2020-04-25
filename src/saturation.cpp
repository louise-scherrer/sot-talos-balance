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

#include "sot/talos_balance/saturation.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

//Size to be aligned\
"-------------------------------------------------------"

#define PROFILE_SATURATION_SOUT_COMPUTATION "Saturation: sOut computation"

#define INPUT_SIGNALS m_xSIN << m_ySIN << m_kSIN << m_xLimSIN << m_yLimSIN

#define OUTPUT_SIGNALS m_yOutSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef Saturation EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Saturation, "Saturation");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
Saturation::Saturation(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(x, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(y, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(k, double),
      CONSTRUCT_SIGNAL_IN(xLim, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(yLim, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(yOut, dynamicgraph::Vector, INPUT_SIGNALS) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(yOut, dynamicgraph::Vector) {
  getProfiler().start(PROFILE_SATURATION_SOUT_COMPUTATION);

  const double &x = m_xSIN(iter)[0];
  s = m_ySIN(iter);
  const double &y = s[0];
  const double &k = m_kSIN(iter);
  const double &xLim = m_xLimSIN(iter)[0];
  const double &yLim = m_yLimSIN(iter)[0];

  double r = y;

  assert(k > 0 && "k must be strictly positive");
  assert(xLim > 0 && "xLim must be strictly positive");
  assert(yLim > 0 && "yLim must be strictly positive");

  if ((x <= -xLim) or (x > xLim)) {
    r = 0.0;
  } else if (-xLim + yLim / k < x and x <= xLim - yLim / k) {
    r = std::min(std::max(y, -yLim), yLim);
  } else if (-xLim < x and x <= -xLim + yLim / k) {
    r = std::min(std::max(y, -k * (x + xLim)), k * (x + xLim));
  } else if (xLim - yLim / k < x and x <= xLim) {
    r = std::min(std::max(y, -yLim + k * (x - xLim + yLim / k)), yLim - k * (x - xLim + yLim / k));
  }

  s[0] = r;

  getProfiler().stop(PROFILE_SATURATION_SOUT_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void Saturation::display(std::ostream &os) const {
  os << "Saturation " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph
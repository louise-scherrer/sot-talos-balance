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

#include "sot/talos_balance/example.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                         "-------------------------------------------------------"
#define PROFILE_EXAMPLE_SUM_COMPUTATION "Example: sum computation                               "
#define PROFILE_EXAMPLE_NBJOINTS_COMPUTATION "Example: nbJoints extraction                           "

#define INPUT_SIGNALS m_firstAddendSIN << m_secondAddendSIN

#define OUTPUT_SIGNALS m_sumSOUT << m_nbJointsSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef Example EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Example, "Example");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
Example::Example(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(firstAddend, double),
      CONSTRUCT_SIGNAL_IN(secondAddend, double),
      CONSTRUCT_SIGNAL_OUT(sum, double, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(nbJoints, int, INPUT_SIGNALS),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &Example::init, docCommandVoid1("Initialize the entity.", "Robot name")));
}

void Example::init(const std::string& robotName) {
  if (!m_firstAddendSIN.isPlugged()) return SEND_MSG("Init failed: signal firstAddend is not plugged", MSG_TYPE_ERROR);
  if (!m_secondAddendSIN.isPlugged())
    return SEND_MSG("Init failed: signal secondAddend is not plugged", MSG_TYPE_ERROR);

  /*std::string & robotName_nonconst = const_cast<std::string &>(robotName);*/
  std::string robotName_nonconst(robotName);

  if (!isNameInRobotUtil(robotName_nonconst)) {
    SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
  } else {
    m_robot_util = getRobotUtil(robotName_nonconst);
    std::cerr << "m_robot_util:" << m_robot_util << std::endl;
  }
  for (unsigned int i = 0; i < 4; i++) {
    std::cout << "Verbosity Level :" << i << std::endl;
    Example::setLoggerVerbosityLevel((dynamicgraph::LoggerVerbosity)i);
    Example::sendMsg("TEST MSG ERROR", dynamicgraph::MSG_TYPE_ERROR);
    Example::sendMsg("TEST MSG DEBUG", dynamicgraph::MSG_TYPE_DEBUG);
    Example::sendMsg("TEST MSG INFO", dynamicgraph::MSG_TYPE_INFO);
    Example::sendMsg("TEST MSG WARNING", dynamicgraph::MSG_TYPE_WARNING);
  }
  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(sum, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_EXAMPLE_SUM_COMPUTATION);

  double firstAddend = m_firstAddendSIN(iter);
  double secondAddend = m_secondAddendSIN(iter);

  s = firstAddend + secondAddend;

  getProfiler().stop(PROFILE_EXAMPLE_SUM_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(nbJoints, int) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal nbJoints before initialization!");
    return s;
  }
  (void)iter;

  getProfiler().start(PROFILE_EXAMPLE_NBJOINTS_COMPUTATION);

  s = int(m_robot_util->m_nbJoints);

  getProfiler().stop(PROFILE_EXAMPLE_NBJOINTS_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void Example::display(std::ostream& os) const {
  os << "Example " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

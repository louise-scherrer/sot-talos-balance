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

#include "sot/talos_balance/simple-reference-frame.hh"

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

// Size to be aligned                                         "-------------------------------------------------------"
#define PROFILE_SIMPLEREFERENCEFRAME_DCM_COMPUTATION "SimpleReferenceFrame: dcm computation          "

#define INPUT_SIGNALS m_footLeftSIN << m_footRightSIN << m_resetSIN

#define OUTPUT_SIGNALS m_referenceFrameSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleReferenceFrame EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleReferenceFrame, "SimpleReferenceFrame");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleReferenceFrame::SimpleReferenceFrame(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(footLeft, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(footRight, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(reset, bool),
      CONSTRUCT_SIGNAL_OUT(referenceFrame, MatrixHomogeneous, m_footLeftSIN << m_footRightSIN << m_resetSIN),
      m_first(true),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);
  m_referenceFrame.setIdentity();

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &SimpleReferenceFrame::init,
                                      docCommandVoid1("Initialize the entity.", "Robot name")));
}

void SimpleReferenceFrame::init(const std::string& robotName) {
  try {
    /* Retrieve m_robot_util informations */
    std::string localName(robotName);
    if (isNameInRobotUtil(localName)) {
      m_robot_util = getRobotUtil(localName);
    } else {
      SEND_ERROR_STREAM_MSG("You should have a robotUtil pointer initialized before");
      return;
    }
  } catch (const std::exception& e) {
    SEND_ERROR_STREAM_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename);
    return;
  }

  m_rightFootSoleXYZ = m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ;

  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(referenceFrame, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal referenceFrame before initialization!");
    return s;
  }

  const MatrixHomogeneous& footLeft = m_footLeftSIN(iter);
  const MatrixHomogeneous& footRight = m_footRightSIN(iter);
  const bool reset = m_resetSIN.isPlugged() ? m_resetSIN(iter) : false;

  if (reset || m_first) {
    Eigen::Vector3d centerTranslation = (footLeft.translation() + footRight.translation()) / 2 + m_rightFootSoleXYZ;
    centerTranslation[2] = 0;

    m_referenceFrame.linear() = footRight.linear();
    m_referenceFrame.translation() = centerTranslation;
    m_first = false;
  }

  s = m_referenceFrame;

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SimpleReferenceFrame::display(std::ostream& os) const {
  os << "SimpleReferenceFrame " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

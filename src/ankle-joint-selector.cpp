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

#include "sot/talos_balance/ankle-joint-selector.hh"

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

#define INPUT_SIGNALS                                                                                               \
  m_phaseSIN << m_rightRollCoupledSIN << m_rightRollDecoupledSIN << m_rightPitchCoupledSIN                          \
             << m_rightPitchDecoupledSIN << m_leftRollCoupledSIN << m_leftRollDecoupledSIN << m_leftPitchCoupledSIN \
             << m_leftPitchDecoupledSIN

#define OUTPUT_SIGNALS \
  m_selecLeftSOUT << m_selecRightSOUT << m_rightRollSOUT << m_rightPitchSOUT << m_leftRollSOUT << m_leftPitchSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef AnkleJointSelector EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AnkleJointSelector, "AnkleJointSelector");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
AnkleJointSelector::AnkleJointSelector(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(phase, int),
      CONSTRUCT_SIGNAL_IN(rightRollCoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rightRollDecoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rightPitchCoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rightPitchDecoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(leftRollCoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(leftRollDecoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(leftPitchCoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(leftPitchDecoupled, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(selecLeft, Flags, m_phaseSIN),
      CONSTRUCT_SIGNAL_OUT(selecRight, Flags, m_phaseSIN),
      CONSTRUCT_SIGNAL_OUT(rightRoll, dynamicgraph::Vector,
                           m_phaseSIN << m_rightRollCoupledSIN << m_rightRollDecoupledSIN),
      CONSTRUCT_SIGNAL_OUT(rightPitch, dynamicgraph::Vector,
                           m_phaseSIN << m_rightPitchCoupledSIN << m_rightPitchDecoupledSIN),
      CONSTRUCT_SIGNAL_OUT(leftRoll, dynamicgraph::Vector,
                           m_phaseSIN << m_leftRollCoupledSIN << m_leftRollDecoupledSIN),
      CONSTRUCT_SIGNAL_OUT(leftPitch, dynamicgraph::Vector,
                           m_phaseSIN << m_leftPitchCoupledSIN << m_leftPitchDecoupledSIN),
      m_zeros(),
      m_ones(true),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &AnkleJointSelector::init,
                                      docCommandVoid1("Initialize the entity.", "Number of joints")));
}

void AnkleJointSelector::init(const int& n) {
  m_n = n;
  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(selecLeft, Flags) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute selecLeft before initialization!");
    return s;
  }

  const int& phase = m_phaseSIN(iter);

  s = phase >= 0 ? m_ones : m_zeros;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(selecRight, Flags) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute selecRight before initialization!");
    return s;
  }

  const int& phase = m_phaseSIN(iter);

  s = phase <= 0 ? m_ones : m_zeros;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(leftRoll, dg::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute leftRoll before initialization!");
    return s;
  }
  if (s.size() != 1) s.resize(1);

  const int& phase = m_phaseSIN(iter);
  const dg::Vector& leftRollCoupled = m_leftRollCoupledSIN(iter);
  const dg::Vector& leftRollDecoupled = m_leftRollDecoupledSIN(iter);

  if (phase > 0)
    s = leftRollDecoupled;
  else if (phase == 0)
    s = leftRollCoupled;
  else
    s.setZero();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(leftPitch, dg::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute leftPitch before initialization!");
    return s;
  }
  if (s.size() != 1) s.resize(1);

  const int& phase = m_phaseSIN(iter);
  const dg::Vector& leftPitchCoupled = m_leftPitchCoupledSIN(iter);
  const dg::Vector& leftPitchDecoupled = m_leftPitchDecoupledSIN(iter);

  if (phase > 0)
    s = leftPitchDecoupled;
  else if (phase == 0)
    s = leftPitchCoupled;
  else
    s.setZero();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rightRoll, dg::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute rightRoll before initialization!");
    return s;
  }
  if (s.size() != 1) s.resize(1);

  const int& phase = m_phaseSIN(iter);
  const dg::Vector& rightRollCoupled = m_rightRollCoupledSIN(iter);
  const dg::Vector& rightRollDecoupled = m_rightRollDecoupledSIN(iter);

  if (phase < 0)
    s = rightRollDecoupled;
  else if (phase == 0)
    s = rightRollCoupled;
  else
    s.setZero();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rightPitch, dg::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute rightPitch before initialization!");
    return s;
  }
  if (s.size() != 1) s.resize(1);

  const int& phase = m_phaseSIN(iter);
  const dg::Vector& rightPitchCoupled = m_rightPitchCoupledSIN(iter);
  const dg::Vector& rightPitchDecoupled = m_rightPitchDecoupledSIN(iter);

  if (phase < 0)
    s = rightPitchDecoupled;
  else if (phase == 0)
    s = rightPitchCoupled;
  else
    s.setZero();

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void AnkleJointSelector::display(std::ostream& os) const {
  os << "AnkleJointSelector " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

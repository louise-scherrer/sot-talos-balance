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

#include "sot/talos_balance/dummy-walking-pattern-generator.hh"

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
#define PROFILE_DUMMYWALKINGPATTERNGENERATOR_DCM_COMPUTATION "DummyWalkingPatternGenerator: dcm computation          "

#define INPUT_SIGNALS                                                                                            \
  m_omegaSIN << m_rhoSIN << m_phaseSIN << m_footLeftSIN << m_footRightSIN << m_waistSIN << m_comSIN << m_vcomSIN \
             << m_acomSIN << m_zmpSIN << m_referenceFrameSIN

#define INNER_SIGNALS m_rfSINNER

#define OUTPUT_SIGNALS                                                                                \
  m_comDesSOUT << m_vcomDesSOUT << m_acomDesSOUT << m_dcmDesSOUT << m_zmpDesSOUT << m_footLeftDesSOUT \
               << m_footRightDesSOUT << m_waistDesSOUT << m_omegaDesSOUT << m_rhoDesSOUT << m_phaseDesSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DummyWalkingPatternGenerator EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DummyWalkingPatternGenerator, "DummyWalkingPatternGenerator");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
DummyWalkingPatternGenerator::DummyWalkingPatternGenerator(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(omega, double),
      CONSTRUCT_SIGNAL_IN(rho, double),
      CONSTRUCT_SIGNAL_IN(phase, int),
      CONSTRUCT_SIGNAL_IN(footLeft, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(footRight, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(waist, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(com, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(vcom, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(acom, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(zmp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(referenceFrame, MatrixHomogeneous),
      CONSTRUCT_SIGNAL_INNER(rf, MatrixHomogeneous, m_referenceFrameSIN),
      CONSTRUCT_SIGNAL_OUT(comDes, dynamicgraph::Vector, m_comSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(vcomDes, dynamicgraph::Vector, m_vcomSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(acomDes, dynamicgraph::Vector, m_acomSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(dcmDes, dynamicgraph::Vector, m_omegaSIN << m_comDesSOUT << m_vcomDesSOUT),
      CONSTRUCT_SIGNAL_OUT(zmpDes, dynamicgraph::Vector,
                           m_omegaSIN << m_comDesSOUT << m_acomDesSOUT << m_rfSINNER << m_zmpSIN),
      CONSTRUCT_SIGNAL_OUT(footLeftDes, MatrixHomogeneous, m_footLeftSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(footRightDes, MatrixHomogeneous, m_footRightSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(waistDes, MatrixHomogeneous, m_waistSIN << m_rfSINNER),
      CONSTRUCT_SIGNAL_OUT(omegaDes, double, m_omegaSIN),
      CONSTRUCT_SIGNAL_OUT(rhoDes, double, m_rhoSIN),
      CONSTRUCT_SIGNAL_OUT(phaseDes, int, m_phaseSIN),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init",
             makeCommandVoid0(*this, &DummyWalkingPatternGenerator::init, docCommandVoid0("Initialize the entity.")));
}

dynamicgraph::Vector DummyWalkingPatternGenerator::actInv(MatrixHomogeneous m, dynamicgraph::Vector v) {
  return m.linear().transpose() * (v - m.translation());
}

MatrixHomogeneous DummyWalkingPatternGenerator::actInv(MatrixHomogeneous m1, MatrixHomogeneous m2) {
  MatrixHomogeneous res;
  res.linear() = m1.linear().transpose() * m2.linear();
  res.translation() = m1.linear().transpose() * (m2.translation() - m1.translation());
  return res;
}

void DummyWalkingPatternGenerator::init() { m_initSucceeded = true; }

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_INNER_FUNCTION(rf, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal rf before initialization!");
    return s;
  }

  s = m_referenceFrameSIN.isPlugged() ? m_referenceFrameSIN(iter) : MatrixHomogeneous::Identity();

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(comDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal comDes before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Vector& com = m_comSIN(iter);
  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  assert(com.size() == 3 && "Unexpected size of signal com");

  s = actInv(referenceFrame, com);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(vcomDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal vcomDes before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Vector& vcom = m_vcomSIN(iter);
  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  assert(vcom.size() == 3 && "Unexpected size of signal vcom");

  s = referenceFrame.linear().transpose() * vcom;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(acomDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal acomDes before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const Vector& acom = m_acomSIN(iter);
  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  assert(acom.size() == 3 && "Unexpected size of signal acom");

  s = referenceFrame.linear().transpose() * acom;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dcmDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dcmDes before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const double& omega = m_omegaSIN(iter);
  const Vector& comDes = m_comDesSOUT(iter);
  const Vector& vcomDes = m_vcomDesSOUT(iter);

  s = comDes + vcomDes / omega;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmpDes, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmpDes before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);

  Eigen::Vector3d zmpDes;

  if (m_zmpSIN.isPlugged()) {
    const Vector& zmp = m_zmpSIN(iter);
    const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

    zmpDes[0] = zmp[0];
    zmpDes[1] = zmp[1];
    zmpDes[2] = zmp.size() > 2 ? zmp[2] : 0.;

    zmpDes = actInv(referenceFrame, zmpDes);
  } else {
    const double& omega = m_omegaSIN(iter);
    const Vector& comDes = m_comDesSOUT(iter);
    const Vector& acomDes = m_acomDesSOUT(iter);

    zmpDes = comDes - acomDes / (omega * omega);
    zmpDes[2] = 0.0;
  }

  s = zmpDes;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(footLeftDes, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal footLeftDes before initialization!");
    return s;
  }

  MatrixHomogeneous footLeft = m_footLeftSIN(iter);

  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  s = actInv(referenceFrame, footLeft);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(footRightDes, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal footRightDes before initialization!");
    return s;
  }

  MatrixHomogeneous footRight = m_footRightSIN(iter);

  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  s = actInv(referenceFrame, footRight);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(waistDes, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal waistDes before initialization!");
    return s;
  }

  const MatrixHomogeneous& waist = m_waistSIN(iter);
  const MatrixHomogeneous& referenceFrame = m_rfSINNER(iter);

  s = actInv(referenceFrame, waist);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(omegaDes, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal omegaDes before initialization!");
    return s;
  }

  s = m_omegaSIN(iter);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rhoDes, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal rhoDes before initialization!");
    return s;
  }

  s = m_rhoSIN(iter);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(phaseDes, int) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal phaseDes before initialization!");
    return s;
  }

  s = m_phaseSIN(iter);
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void DummyWalkingPatternGenerator::display(std::ostream& os) const {
  os << "DummyWalkingPatternGenerator " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

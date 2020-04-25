/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/talos_balance/talos-control-manager.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>

using namespace sot::talos_balance;

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace std;
using namespace dg::sot::talos_balance;

// Size to be aligned                          "-------------------------------------------------------"
#define PROFILE_PWM_DESIRED_COMPUTATION "Control manager                                        "
#define PROFILE_DYNAMIC_GRAPH_PERIOD "Control period                                         "

#define INPUT_SIGNALS m_u_maxSIN
#define OUTPUT_SIGNALS m_uSOUT << m_u_safeSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef TalosControlManager EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TalosControlManager, "TalosControlManager");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
TalosControlManager::TalosControlManager(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(u_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(u, dynamicgraph::Vector, m_u_maxSIN),
      CONSTRUCT_SIGNAL_OUT(u_safe, dynamicgraph::Vector, m_uSOUT << m_u_maxSIN),
      m_robot_util(RefVoidRobotUtil()),
      m_initSucceeded(false),
      m_emergency_stop_triggered(false),
      m_is_first_iter(true),
      m_iter(0),
      m_sleep_time(0.0) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &TalosControlManager::init,
                                      docCommandVoid2("Initialize the entity.", "Time period in seconds (double)",
                                                      "Robot reference (string)")));
  addCommand(
      "addCtrlMode",
      makeCommandVoid1(*this, &TalosControlManager::addCtrlMode,
                       docCommandVoid1("Create an input signal with name 'ctrl_x' where x is the specified name.",
                                       "Name (string)")));

  addCommand("ctrlModes", makeCommandVoid0(*this, &TalosControlManager::ctrlModes,
                                           docCommandVoid0("Get a list of all the available control modes.")));

  addCommand("setCtrlMode", makeCommandVoid2(*this, &TalosControlManager::setCtrlMode,
                                             docCommandVoid2("Set the control mode for a joint.",
                                                             "(string) joint name", "(string) control mode")));

  addCommand("getCtrlMode",
             makeCommandVoid1(*this, &TalosControlManager::getCtrlMode,
                              docCommandVoid1("Get the control mode of a joint.", "(string) joint name")));

  addCommand("resetProfiler",
             makeCommandVoid0(
                 *this, &TalosControlManager::resetProfiler,
                 docCommandVoid0("Reset the statistics computed by the profiler (print this entity to see them).")));

  addCommand(
      "addEmergencyStopSIN",
      makeCommandVoid1(
          *this, &TalosControlManager::addEmergencyStopSIN,
          docCommandVoid1("Add emergency signal input from another entity that can stop the control if necessary.",
                          "(string) signal name : 'emergencyStop_' + name")));

  addCommand("isEmergencyStopTriggered",
             makeDirectGetter(*this, &m_emergency_stop_triggered,
                              docDirectGetter("Check whether emergency stop is triggered", "bool")));
}

void TalosControlManager::init(const double& dt, const std::string& robotRef) {
  if (dt <= 0.0) return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
  m_dt = dt;
  m_emergency_stop_triggered = false;
  m_initSucceeded = true;
  vector<string> package_dirs;
  std::string localName(robotRef);
  if (!isNameInRobotUtil(localName)) {
    SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
  } else {
    m_robot_util = getRobotUtil(localName);
  }
  m_numDofs = m_robot_util->m_nbJoints + 6;

  // TalosControlManager::setLoggerVerbosityLevel((dynamicgraph::LoggerVerbosity) 4);
  m_jointCtrlModes_current.resize(m_numDofs);
  m_jointCtrlModes_previous.resize(m_numDofs);
  m_jointCtrlModesCountDown.resize(m_numDofs, 0);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_MSG("Cannot compute signal u before initialization!", MSG_TYPE_WARNING);
    return s;
  }

  if (m_is_first_iter) m_is_first_iter = false;

  if (s.size() != (Eigen::VectorXd::Index)m_numDofs) s.resize(m_numDofs);

  {
    // trigger computation of all ctrl inputs
    for (unsigned int i = 0; i < m_ctrlInputsSIN.size(); i++) (*m_ctrlInputsSIN[i])(iter);

    int cm_id, cm_id_prev;
    for (unsigned int i = 0; i < m_numDofs; i++) {
      cm_id = m_jointCtrlModes_current[i].id;
      if (cm_id < 0) {
        SEND_MSG("You forgot to set the control mode of joint " + toString(i), MSG_TYPE_ERROR_STREAM);
        continue;
      }

      const dynamicgraph::Vector& ctrl = (*m_ctrlInputsSIN[cm_id])(iter);
      assert(ctrl.size() == m_numDofs);

      if (m_jointCtrlModesCountDown[i] == 0)
        s(i) = ctrl(i);
      else {
        cm_id_prev = m_jointCtrlModes_previous[i].id;
        const dynamicgraph::Vector& ctrl_prev = (*m_ctrlInputsSIN[cm_id_prev])(iter);
        assert(ctrl_prev.size() == m_numDofs);

        double alpha = m_jointCtrlModesCountDown[i] / CTRL_MODE_TRANSITION_TIME_STEP;
        //              SEND_MSG("Joint "+toString(i)+" changing ctrl mode from "+toString(cm_id_prev)+
        //                       " to "+toString(cm_id)+" alpha="+toString(alpha),MSG_TYPE_DEBUG);
        s(i) = alpha * ctrl_prev(i) + (1 - alpha) * ctrl(i);
        m_jointCtrlModesCountDown[i]--;

        if (m_jointCtrlModesCountDown[i] == 0) {
          SEND_MSG(
              "Joint " + toString(i) + " changed ctrl mode from " + toString(cm_id_prev) + " to " + toString(cm_id),
              MSG_TYPE_INFO);
          updateJointCtrlModesOutputSignal();
        }
      }
    }
  }

  // usleep(1e6*m_sleep_time);
  // if(m_sleep_time>=0.1)
  // {
  //   for(unsigned int i=0; i<m_ctrlInputsSIN.size(); i++)
  //   {
  //     const dynamicgraph::Vector& ctrl = (*m_ctrlInputsSIN[i])(iter);
  //     SEND_MSG(toString(iter)+") tau =" +toString(ctrl,1,4," ")+m_ctrlModes[i], MSG_TYPE_ERROR);
  //   }
  // }

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(u_safe, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal u_safe before initialization!");
    return s;
  }
  if (s.size() != (Eigen::VectorXd::Index)m_numDofs) s.resize(m_numDofs);

  const dynamicgraph::Vector& u = m_uSOUT(iter);
  const dynamicgraph::Vector& ctrl_max = m_u_maxSIN(iter);

  for (size_t i = 0; i < m_emergencyStopVector.size(); i++) {
    if ((*m_emergencyStopVector[i]).isPlugged() && (*m_emergencyStopVector[i])(iter)) {
      m_emergency_stop_triggered = true;
      SEND_MSG("t = " + toString(iter) + ": Emergency Stop has been triggered by an external entity: " +
                   (*m_emergencyStopVector[i]).getName(),
               MSG_TYPE_ERROR);
    }
  }

  if (!m_emergency_stop_triggered) {
    for (unsigned int i = 0; i < m_numDofs; i++) {
      if (fabs(u(i)) > ctrl_max(i)) {
        m_emergency_stop_triggered = true;
        SEND_MSG("t = " + toString(iter) + ": Joint " + toString(i) +
                     " desired control is too large: " + toString(u(i)) + " > " + toString(ctrl_max(i)),
                 MSG_TYPE_ERROR_STREAM);
        break;
      }
    }
  }

  s = u;

  if (m_emergency_stop_triggered) s.setZero();

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

void TalosControlManager::addCtrlMode(const string& name) {
  // check there is no other control mode with the same name
  for (unsigned int i = 0; i < m_ctrlModes.size(); i++)
    if (name == m_ctrlModes[i]) return SEND_MSG("It already exists a control mode with name " + name, MSG_TYPE_ERROR);

  // create a new input signal to read the new control
  m_ctrlInputsSIN.push_back(new SignalPtr<dynamicgraph::Vector, int>(
      NULL, getClassName() + "(" + getName() + ")::input(dynamicgraph::Vector)::ctrl_" + name));

  // create a new output signal to specify which joints are controlled with the new
  // control mode
  m_jointsCtrlModesSOUT.push_back(new Signal<dynamicgraph::Vector, int>(
      getClassName() + "(" + getName() + ")::output(dynamicgraph::Vector)::joints_ctrl_mode_" + name));

  // add the new control mode to the list of available control modes
  m_ctrlModes.push_back(name);

  // register the new signals and add the new signal dependecy
  Eigen::VectorXd::Index i = m_ctrlModes.size() - 1;
  m_uSOUT.addDependency(*m_ctrlInputsSIN[i]);
  m_u_safeSOUT.addDependency(*m_ctrlInputsSIN[i]);
  Entity::signalRegistration(*m_ctrlInputsSIN[i]);
  Entity::signalRegistration(*m_jointsCtrlModesSOUT[i]);
  updateJointCtrlModesOutputSignal();
}

void TalosControlManager::ctrlModes() { SEND_MSG(toString(m_ctrlModes), MSG_TYPE_INFO); }

void TalosControlManager::setCtrlMode(const string& jointName, const string& ctrlMode) {
  CtrlMode cm;
  if (convertStringToCtrlMode(ctrlMode, cm) == false) return;

  if (jointName == "all") {
    for (unsigned int i = 0; i < m_numDofs; i++) setCtrlMode(i, cm);
  } else {
    // decompose strings like "rk-rhp-lhp-..."
    std::stringstream ss(jointName);
    std::string item;
    std::list<int> jIdList;
    unsigned int i;
    while (std::getline(ss, item, '-')) {
      SEND_MSG("parsed joint : " + item, MSG_TYPE_INFO);
      if (convertJointNameToJointId(item, i)) jIdList.push_back(i);
    }
    for (std::list<int>::iterator it = jIdList.begin(); it != jIdList.end(); ++it) setCtrlMode(*it, cm);
  }
  updateJointCtrlModesOutputSignal();
}

void TalosControlManager::setCtrlMode(const int jid, const CtrlMode& cm) {
  if (m_jointCtrlModesCountDown[jid] != 0)
    return SEND_MSG("Cannot change control mode of joint " + toString(jid) +
                        " because its previous ctrl mode transition has not terminated yet: " +
                        toString(m_jointCtrlModesCountDown[jid]),
                    MSG_TYPE_ERROR);

  if (cm.id == m_jointCtrlModes_current[jid].id)
    return SEND_MSG(
        "Cannot change control mode of joint " + toString(jid) + " because it has already the specified ctrl mode",
        MSG_TYPE_ERROR);

  if (m_jointCtrlModes_current[jid].id < 0) {
    // first setting of the control mode
    m_jointCtrlModes_previous[jid] = cm;
    m_jointCtrlModes_current[jid] = cm;
  } else {
    m_jointCtrlModesCountDown[jid] = CTRL_MODE_TRANSITION_TIME_STEP;
    m_jointCtrlModes_previous[jid] = m_jointCtrlModes_current[jid];
    m_jointCtrlModes_current[jid] = cm;
  }
}

void TalosControlManager::getCtrlMode(const std::string& jointName) {
  if (jointName == "all") {
    stringstream ss;
    for (unsigned int i = 0; i < m_numDofs; i++) ss << toString(i) << " " << m_jointCtrlModes_current[i] << "; ";
    SEND_MSG(ss.str(), MSG_TYPE_INFO);
    return;
  }

  unsigned int i;
  if (convertJointNameToJointId(jointName, i) == false) return;
  SEND_MSG("The control mode of joint " + jointName + " is " + m_jointCtrlModes_current[i].name, MSG_TYPE_INFO);
}

void TalosControlManager::resetProfiler() {
  getProfiler().reset_all();
  getStatistics().reset_all();
}

//      void TalosControlManager::setStreamPrintPeriod(const double & s)
//      {
//        getLogger().setStreamPrintPeriod(s);
//      }

void TalosControlManager::setSleepTime(const double& seconds) {
  if (seconds < 0.0) return SEND_MSG("Sleep time cannot be negative!", MSG_TYPE_ERROR);
  m_sleep_time = seconds;
}

void TalosControlManager::addEmergencyStopSIN(const string& name) {
  SEND_MSG("New emergency signal input emergencyStop_" + name + " created", MSG_TYPE_INFO);
  // create a new input signal
  m_emergencyStopVector.push_back(
      new SignalPtr<bool, int>(NULL, getClassName() + "(" + getName() + ")::input(bool)::emergencyStop_" + name));

  // register the new signals and add the new signal dependecy
  Eigen::Index i = m_emergencyStopVector.size() - 1;
  m_u_safeSOUT.addDependency(*m_emergencyStopVector[i]);
  Entity::signalRegistration(*m_emergencyStopVector[i]);
}

/* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */

void TalosControlManager::updateJointCtrlModesOutputSignal() {
  if (m_numDofs == 0) {
    SEND_MSG("You should call init first. The size of the vector is unknown.", MSG_TYPE_ERROR);
    return;
  }

  dynamicgraph::Vector cm(m_numDofs);
  for (unsigned int i = 0; i < m_jointsCtrlModesSOUT.size(); i++) {
    for (unsigned int j = 0; j < m_numDofs; j++) {
      cm(j) = 0;
      if ((unsigned int)m_jointCtrlModes_current[j].id == i) cm(j) = 1;

      // during the transition between two ctrl modes they both result active
      if (m_jointCtrlModesCountDown[j] > 0 && (unsigned int)m_jointCtrlModes_previous[j].id == i) cm(j) = 1;
    }
    m_jointsCtrlModesSOUT[i]->setConstant(cm);
  }
}

bool TalosControlManager::convertStringToCtrlMode(const std::string& name, CtrlMode& cm) {
  // Check if the ctrl mode name exists
  for (unsigned int i = 0; i < m_ctrlModes.size(); i++)
    if (name == m_ctrlModes[i]) {
      cm = CtrlMode(i, name);
      return true;
    }
  SEND_MSG("The specified control mode does not exist", MSG_TYPE_ERROR);
  SEND_MSG("Possible control modes are: " + toString(m_ctrlModes), MSG_TYPE_INFO);
  return false;
}

bool TalosControlManager::convertJointNameToJointId(const std::string& name, unsigned int& id) {
  // Check if the joint name exists
  int jid = int(m_robot_util->get_id_from_name(name));  // cast needed due to bug in robot-utils
  jid += 6;                                             // Take into account the FF
  if (jid < 0) {
    SEND_MSG("The specified joint name does not exist: " + name, MSG_TYPE_ERROR);
    std::stringstream ss;
    for (size_t it = 0; it < m_numDofs; it++) ss << toString(it) << ", ";
    SEND_MSG("Possible joint names are: " + ss.str(), MSG_TYPE_INFO);
    return false;
  }
  id = (unsigned int)jid;
  return true;
}

/*
      bool TalosControlManager::isJointInRange(unsigned int id, double q)
      {
        const JointLimits & JL = m_robot_util->get_joint_limits_from_id((Index)id);

        double jl= JL.lower;
        if(q<jl)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is smaller than lower limit: "+toString(jl),MSG_TYPE_ERROR);
          return false;
        }
        double ju = JL.upper;
        if(q>ju)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is larger than upper limit: "+toString(ju),MSG_TYPE_ERROR);
          return false;
        }
        return true;
      }
*/

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void TalosControlManager::display(std::ostream& os) const {
  os << "TalosControlManager " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

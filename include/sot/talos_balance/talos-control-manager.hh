/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_control_manager_H__
#define __sot_torque_control_control_manager_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(__sot_torque_control_control_manager_H__)
#define TALOS_CONTROL_MANAGER_EXPORT __declspec(dllexport)
#else
#define TALOS_CONTROL_MANAGER_EXPORT __declspec(dllimport)
#endif
#else
#define TALOS_CONTROL_MANAGER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Number of time step to transition from one ctrl mode to another
#define CTRL_MODE_TRANSITION_TIME_STEP 1000.0

class CtrlMode {
 public:
  int id;
  std::string name;

  CtrlMode() : id(-1), name("None") {}
  CtrlMode(int id, const std::string& name) : id(id), name(name) {}
};

std::ostream& operator<<(std::ostream& os, const CtrlMode& s) {
  os << s.id << "_" << s.name;
  return os;
}

class TALOS_CONTROL_MANAGER_EXPORT TalosControlManager : public ::dynamicgraph::Entity {
  typedef Eigen::VectorXd::Index Index;
  typedef TalosControlManager EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  TalosControlManager(const std::string& name);

  /// Initialize
  /// @param dt: control interval
  /// @param urdfFile: path to the URDF model of the robot
  void init(const double& dt, const std::string& robotRef);

  /* --- SIGNALS --- */
  std::vector<dynamicgraph::SignalPtr<dynamicgraph::Vector, int>*> m_ctrlInputsSIN;
  std::vector<dynamicgraph::SignalPtr<bool, int>*>
      m_emergencyStopVector;  /// emergency stop inputs. If one is true, control is set to zero forever
  std::vector<dynamicgraph::Signal<dynamicgraph::Vector, int>*> m_jointsCtrlModesSOUT;

  DECLARE_SIGNAL_IN(u_max, dynamicgraph::Vector);    /// max motor control
  DECLARE_SIGNAL_OUT(u, dynamicgraph::Vector);       /// raw motor control
  DECLARE_SIGNAL_OUT(u_safe, dynamicgraph::Vector);  /// safe motor control

  /* --- COMMANDS --- */

  /// Commands related to the control mode.
  void addCtrlMode(const std::string& name);
  void ctrlModes();
  void getCtrlMode(const std::string& jointName);
  void setCtrlMode(const std::string& jointName, const std::string& ctrlMode);
  void setCtrlMode(const int jid, const CtrlMode& cm);

  void resetProfiler();

  /// Commands related to joint name and joint id
  // void setNameToId(const std::string& jointName, const double & jointId);
  // void setJointLimitsFromId(const double &jointId, const double &lq, const double &uq);

  /// Set the mapping between urdf and sot.
  // void setJoints(const dynamicgraph::Vector &);

  // void setStreamPrintPeriod(const double & s);

  void setSleepTime(const double& seconds);
  void addEmergencyStopSIN(const std::string& name);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  RobotUtilShrPtr m_robot_util;
  size_t m_numDofs;
  bool m_initSucceeded;             /// true if the entity has been successfully initialized
  double m_dt;                      /// control loop time period
  bool m_emergency_stop_triggered;  /// true if an emergency condition as been triggered either by an other entity, or
                                    /// by control limit violation
  bool m_is_first_iter;             /// true at the first iteration, false otherwise
  int m_iter;
  double m_sleep_time;  /// time to sleep at every iteration (to slow down simulation)

  std::vector<std::string> m_ctrlModes;             /// existing control modes
  std::vector<CtrlMode> m_jointCtrlModes_current;   /// control mode of the joints
  std::vector<CtrlMode> m_jointCtrlModes_previous;  /// previous control mode of the joints
  std::vector<int> m_jointCtrlModesCountDown;       /// counters used for the transition between two ctrl modes

  bool convertStringToCtrlMode(const std::string& name, CtrlMode& cm);
  bool convertJointNameToJointId(const std::string& name, unsigned int& id);
  // bool isJointInRange(unsigned int id, double q);
  void updateJointCtrlModesOutputSignal();

};  // class TalosControlManager

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_control_manager_H__

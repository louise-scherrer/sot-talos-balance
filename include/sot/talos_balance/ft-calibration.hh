/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 */

#ifndef __sot_talos_balance_ft_calibration_H__
#define __sot_talos_balance_ft_calibration_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(__sot_talos_balance_ft_calibration_H__)
#define SOTFTCALIBRATION_EXPORT __declspec(dllexport)
#else
#define SOTFTCALIBRATION_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFTCALIBRATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <dynamic-graph/signal-helper.h>
#include <dynamic-graph/real-time-logger.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"

#include <sot/talos_balance/robot/robot-wrapper.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTFTCALIBRATION_EXPORT FtCalibration : public ::dynamicgraph::Entity {
  // typedef FtCalibration EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  FtCalibration(const std::string &name);
  /// Initialize
  void init(const std::string &robotRef);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(right_foot_force_in, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(left_foot_force_in, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_force_out, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_force_out, dynamicgraph::Vector);

  /* --- COMMANDS --- */

  /// Commands for setting the feet weight
  void setRightFootWeight(const double &rightW);
  void setLeftFootWeight(const double &leftW);

  /// Command to calibrate the foot sensors when the robot is standing in the air with horizontal feet
  void calibrateFeetSensor();

  void displayRobotUtil();

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

  /* --- TYPEDEFS ---- */
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

 protected:
  RobotUtilShrPtr m_robot_util;
  int m_right_calibration_iter =
      -1;  /// Number of iteration left for calibration (-1= not cailbrated, 0=caibration done)
  int m_left_calibration_iter =
      -1;                      /// Number of iteration left for calibration (-1= not cailbrated, 0=caibration done)
  Vector6d m_right_FT_offset;  /// Offset or bias to be removed from Right FT sensor
  Vector6d m_left_FT_offset;   /// Offset or bias to be removed from Left FT sensor
  Vector6d m_right_FT_offset_calibration_sum;  /// Variable used durring average computation of the offset
  Vector6d m_left_FT_offset_calibration_sum;   /// Variable used durring average computation of the offset
  bool m_initSucceeded;                        /// true if the entity has been successfully initialized
  Vector6d m_right_foot_weight;                // weight of the right feet underneath the ft sensor
  Vector6d m_left_foot_weight;                 // weight of the left feet underneath the ft sensor

};  // class FtCalibration

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_ft_calibration_H__

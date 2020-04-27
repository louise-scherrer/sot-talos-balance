/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Fanny Risbourg
 * This file is part of sot-talos-balance.
 * See license file.
 */

#ifndef __sot_talos_balance_admittance_controller_end_effector_H__
#define __sot_talos_balance_admittance_controller_end_effector_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(admittance_controller_end_effector_EXPORTS)
#define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT __declspec(dllexport)
#else
#define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT __declspec(dllimport)
#endif
#else
#define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include <sot/core/robot-utils.hh>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/**
 * @brief  Admittance controller for an upper body end-effector (right or left
 *         wrist)
 *
 *  This entity computes a velocity reference for an end-effector based
 *  on the force error in the world frame :
 *  w_dq = integral(Kp(w_forceDes-w_force)) + Kd (w_dq)
 *
 */
class ADMITTANCECONTROLLERENDEFFECTOR_EXPORT AdmittanceControllerEndEffector : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  AdmittanceControllerEndEffector(const std::string &name);

  /* --- SIGNALS --- */
  /// \brief  Gain (6d) for the integration of the error on the force
  DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);
  /// \brief  Derivative gain (6d) for the error on the force
  DECLARE_SIGNAL_IN(Kd, dynamicgraph::Vector);
  /// \brief  Value of the saturation to apply on the velocity output
  DECLARE_SIGNAL_IN(dqSaturation, dynamicgraph::Vector);
  /// \brief  6d force given by the sensor in its local frame
  DECLARE_SIGNAL_IN(force, dynamicgraph::Vector);
  /// \brief  6d desired force of the end-effector in the world frame
  DECLARE_SIGNAL_IN(w_forceDes, dynamicgraph::Vector);
  /// \brief  Current joint configuration of the robot
  DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);

  /// \brief  6d force given by the sensor in the world frame
  DECLARE_SIGNAL_INNER(w_force, dynamicgraph::Vector);
  /// \brief  Internal intergration computed in the world frame
  DECLARE_SIGNAL_INNER(w_dq, dynamicgraph::Vector);

  /// \brief  Velocity reference for the end-effector in the local frame
  DECLARE_SIGNAL_OUT(dq, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /**
   * @brief      Initialize the entity
   *
   * @param[in]  dt  Time step of the control
   * @param[in]  sensorFrameName  Name of the force sensor of the end-effector
   *             used in the pinocchio model
   * @param[in]  endeffectorName  Name of the endEffectorJoint
   */
  void init(const double &dt, const std::string &sensorFrameName, const std::string &endeffectorName);

  /**
   * @brief      Reset the velocity
   */
  void resetDq();

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

 protected:
  /// Dimension of the force signals and of the output
  int m_n;
  /// True if the entity has been successfully initialized
  bool m_initSucceeded;
  /// Internal state
  dynamicgraph::Vector m_w_dq;
  /// Time step of the control
  double m_dt;
  // Weight of the end-effector
  double m_mass;

  /// Robot Util instance to get the sensor frame
  RobotUtilShrPtr m_robot_util;
  /// Pinocchio robot model
  pinocchio::Model m_model;
  /// Pinocchio robot data
  pinocchio::Data *m_data;
  /// Id of the force sensor frame
  pinocchio::FrameIndex m_sensorFrameId;
  /// Id of the joint of the end-effector
  pinocchio::JointIndex m_endEffectorId;

};  // class AdmittanceControllerEndEffector

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_admittance_controller_end_effector_H__

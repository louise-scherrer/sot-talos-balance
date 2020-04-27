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

#ifndef __sot_talos_balance_distribute_wrench_H__
#define __sot_talos_balance_distribute_wrench_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(position_controller_EXPORTS)
#define DISTRIBUTE_WRENCH_EXPORT __declspec(dllexport)
#else
#define DISTRIBUTE_WRENCH_EXPORT __declspec(dllimport)
#endif
#else
#define DISTRIBUTE_WRENCH_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <dynamic-graph/signal-helper.h>

#include <map>
#include "boost/assign.hpp"
#include <sot/core/robot-utils.hh>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <eiquadprog/eiquadprog-fast.hpp>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class DISTRIBUTE_WRENCH_EXPORT DistributeWrench : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  DistributeWrench(const std::string& name);

  void init(const std::string& robotName);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(wrenchDes, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rho, double);
  DECLARE_SIGNAL_IN(phase, int);
  DECLARE_SIGNAL_IN(frictionCoefficient, double);

  DECLARE_SIGNAL_IN(wSum, double);
  DECLARE_SIGNAL_IN(wNorm, double);
  DECLARE_SIGNAL_IN(wRatio, double);
  DECLARE_SIGNAL_IN(wAnkle, dynamicgraph::Vector);

  DECLARE_SIGNAL_INNER(kinematics_computations, int);
  DECLARE_SIGNAL_INNER(qp_computations, int);

  DECLARE_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ankleWrenchLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(surfaceWrenchLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(copLeft, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ankleWrenchRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(surfaceWrenchRight, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(copRight, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmpRef, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(emergencyStop, bool);

 public:
  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  Eigen::Vector3d computeCoP(const dynamicgraph::Vector& wrench, const pinocchio::SE3& pose) const;

  void set_right_foot_sizes(const dynamicgraph::Vector& s);
  void set_left_foot_sizes(const dynamicgraph::Vector& s);

  double m_eps;

 protected:
  bool m_initSucceeded;      /// true if the entity has been successfully initialized
  pinocchio::Model m_model;  /// Pinocchio robot model
  pinocchio::Data m_data;    /// Pinocchio robot data
  RobotUtilShrPtr m_robot_util;

  //        pinocchio::SE3 m_ankle_M_ftSens; /// ankle to F/T sensor transformation
  pinocchio::SE3 m_ankle_M_sole;  /// ankle to sole transformation

  pinocchio::FrameIndex m_left_foot_id;
  pinocchio::FrameIndex m_right_foot_id;

  pinocchio::SE3 m_contactLeft;
  pinocchio::SE3 m_contactRight;

  Eigen::Matrix<double, 6, 1> m_wrenchLeft;
  Eigen::Matrix<double, 6, 1> m_wrenchRight;

  Eigen::Vector4d m_left_foot_sizes;   /// sizes of the left foot (pos x, neg x, pos y, neg y)
  Eigen::Vector4d m_right_foot_sizes;  /// sizes of the left foot (pos x, neg x, pos y, neg y)

  void computeWrenchFaceMatrix(const double mu);
  Eigen::Matrix<double, 16, 6> m_wrenchFaceMatrix;  // for modelling contact

  eiquadprog::solvers::EiquadprogFast m_qp1;  // saturate wrench
  eiquadprog::solvers::EiquadprogFast m_qp2;  // distribute wrench

  // QP problem matrices -- SSP
  Eigen::MatrixXd m_Q1;
  Eigen::VectorXd m_C1;

  Eigen::MatrixXd m_Aeq1;
  Eigen::VectorXd m_Beq1;

  Eigen::MatrixXd m_Aineq1;
  Eigen::VectorXd m_Bineq1;

  Eigen::VectorXd m_result1;

  // QP problem matrices -- DSP
  Eigen::MatrixXd m_Q2;
  Eigen::VectorXd m_C2;

  Eigen::MatrixXd m_Aeq2;
  Eigen::VectorXd m_Beq2;

  Eigen::MatrixXd m_Aineq2;
  Eigen::VectorXd m_Bineq2;

  Eigen::VectorXd m_result2;

  double m_wSum;
  double m_wNorm;
  double m_wRatio;
  Eigen::VectorXd m_wAnkle;

  void distributeWrench(const Eigen::VectorXd& wrenchDes, const double rho, const double mu);
  void saturateWrench(const Eigen::VectorXd& wrenchDes, const int phase, const double mu);

  bool m_emergency_stop_triggered;
};  // class DistributeWrench

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_distribute_wrench_H__

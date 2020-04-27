/*
 * Copyright 2019,
 * LAAS-CNRS,
 * Gepetto team
 *
 * This file is part of sot-talos-balance.
 * See license file
 */

#ifndef __sot_talos_balance_dcm_estimator_H__
#define __sot_talos_balance_dcm_estimator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(dcm_estimator_EXPORTS)
#define DCMESTIMATOR_EXPORT __declspec(dllexport)
#else
#define DCMESTIMATOR_EXPORT __declspec(dllimport)
#endif
#else
#define DCMESTIMATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <map>
#include "boost/assign.hpp"
#include <boost/math/distributions/normal.hpp>  // for normal_distribution

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <sot/core/robot-utils.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class DCMESTIMATOR_EXPORT DcmEstimator : public ::dynamicgraph::Entity {
  typedef pinocchio::SE3 SE3;
  typedef Eigen::Vector2d Vector2;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Vector4d Vector4;
  typedef Vector6d Vector6;
  typedef Vector7d Vector7;
  typedef Eigen::Matrix3d Matrix3;
  typedef boost::math::normal normal;

  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  DcmEstimator(const std::string& name);

  void init(const double& dt, const std::string& urdfFile);
  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(v, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(c, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dc, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  RobotUtilShrPtr m_robot_util;
  pinocchio::Data m_data;    /// Pinocchio robot data
  Eigen::VectorXd m_q_pin;   /// robot configuration according to pinocchio convention
  Eigen::VectorXd m_v_pin;   /// robot velocities according to pinocchio convention
  double m_dt;               /// sampling time step
  pinocchio::Model m_model;  /// Pinocchio robot model

};  // class DCMEstimator

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_dcm_estimator_H__

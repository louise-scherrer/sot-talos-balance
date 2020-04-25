# Copyright 2018, Gepetto team, LAAS-CNRS


SET(plugins
  example
  delay
  boolean-identity
  round-double-to-int
  int-identity
  pose-quaternion-to-matrix-homo
  euler-to-quat
  quat-to-euler
  state-transformation
  simple-reference-frame
  simple-state-integrator
  dummy-walking-pattern-generator
  simple-zmp-estimator
  simple-distribute-wrench
  simple-pid
  simple-pidd
  simple-controller-6d
  distribute-wrench
  dcm-com-controller
  dcm-controller
  dummy-dcm-estimator
  com-admittance-controller
  simple-admittance-controller
  coupled-admittance-controller
  admittance-controller-end-effector
  ankle-admittance-controller
  foot-force-difference-controller
  joint-position-controller
  nd-trajectory-generator
  talos-base-estimator
  talos-control-manager
  dcm-estimator
  qualisys-client
  ft-calibration
  ft-wrist-calibration
  ankle-joint-selector
  saturation
  hip-flexibility-compensation
  )

# set(feature-task_deps feature-generic task)


FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin})

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${PROJECT_NAME} ${${LIBRARY_NAME}_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    SOT_TALOS_BALANCE_PYTHON_MODULE(${PYTHON_LIBRARY_NAME}
      ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

TARGET_LINK_LIBRARIES(distribute-wrench eiquadprog::eiquadprog)

IF(BUILD_PYTHON_INTERFACE)
  SOT_TALOS_BALANCE_PYTHON_MODULE("" ${PROJECT_NAME} wrap)

  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PY_NAME}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${PY_NAME}
    )
ENDIF(BUILD_PYTHON_INTERFACE)

# main python module
SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  main.py
  create_entities_utils.py
  meta_task_config.py
  meta_task_joint.py
  meta_task_pose.py
  motor_parameters.py
  )

# utils submodule
SET(${PROJECT_NAME}_UTILS_PYTHON_FILES
  __init__.py
  control_utils.py
  plot_utils.py
  sot_utils.py
  filter_utils.py
  run_test_utils.py
  gazebo_utils.py
  )

# talos submodule
SET(${PROJECT_NAME}_TALOS_PYTHON_FILES
  __init__.py
  admittance_ctrl_conf.py
  balance_ctrl_conf.py
  balance_ctrl_sim_conf.py
  base_estimator_conf.py
  base_estimator_sim_conf.py
  distribute_conf.py
  control_manager_conf.py
  control_manager_sim_conf.py
  current_controller_conf.py
  current_controller_sim_conf.py
  force_offset_conf.py
  force_torque_estimator_conf.py
  ft_calibration_conf.py
  ft_wrist_calibration_conf.py
  joint_torque_controller_conf.py
  joint_torque_controller_sim_conf.py
  parameter_server_conf.py
  hip_flexibility_compensation_conf.py
  )

# simulation files
SET(${PROJECT_NAME}_SIMULATION_FILES
  __init__.py
  test_dcm_zmp_control.py
  appli_dcm_zmp_control.py
  test_dcm_zmp_control_distribute.py
  appli_dcm_zmp_control_distribute.py
  test_dcm_zmp_control_ffdc.py
  appli_dcm_zmp_control_ffdc.py
  test_ffSubscriber.py
  appli_ffSubscriber.py
  test_zmpEstimator.py
  appli_zmpEstimator.py
  test_dcmComControl.py
  appli_dcmComControl.py
  test_dcmComZmpControl.py
  appli_dcmComZmpControl.py
  test_dcmZmpControl.py
  appli_dcmZmpControl.py
  test_dcmZmpControl_file.py
  appli_dcmZmpControl_file.py
  test_comAdmittance.py
  test_admittance_single_joint.py
  appli_admittance_single_joint.py
  test_admittance_end_effector.py
  appli_admittance_end_effector.py
  test_simple_ankle_admittance.py
  appli_simple_ankle_admittance.py
  test_admittance_single_joint_velocity_based.py
  appli_admittance_single_joint_velocity_based.py
  test_singleTraj.py
  test_jointTrajGen.py
  test_jointControl.py
  test_COMTraj.py
  appli_COMTraj.py
  test_COMTraj_tracer.py
  test_param_server.py
  test_dcm_estimator.py
  appli_dcm_estimator.py
  test_hip_flexibility_compensation.py
  appli_hip_flexibility_compensation.py
  )

IF(BUILD_PYTHON_INTERFACE)
  FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(${PY_NAME} ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(${PY_NAME}/talos ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(${PY_NAME}/utils ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
    PYTHON_INSTALL_ON_SITE(${PY_NAME}/test ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
ENDIF(BUILD_PYTHON_INTERFACE)

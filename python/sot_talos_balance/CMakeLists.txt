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

FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_BUILD(. ${python})
  INSTALL(FILES ${python} DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME})
ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})

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

FOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})
  PYTHON_BUILD(utils ${python})
  INSTALL(FILES utils/${python} DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/utils)
ENDFOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})


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

FOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})
  PYTHON_BUILD(talos ${python})
  INSTALL(FILES talos/${python} DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/talos)
ENDFOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})

IF(BUILD_TEST)
  ADD_SUBDIRECTORY(test)
ENDIF(BUILD_TEST)


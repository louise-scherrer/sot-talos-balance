SET(${PROJECT_NAME}_PYTHON_TESTS
  test_round_double_to_int
  test_foot_force_difference_controller
  test_ankle_joint_selector
  test_base_estimator
  test_com_admittance
  test_com_estimation
  test_control
  test_ctrl_manager
  test_dcm_controller
  test_dcm_controller_talos
  test_dcm_distribute
  test_dcm_estimator
  test_distribute
  test_dummy_dcm_estimator
  test_dummy_wpgen
  test_eulerquat
  test_example
  test_ft_calibration
  test_saturation
  test_simple_admittance
  test_simple_distribute
  test_simple_zmp_estimator
  test_simple_state_integrator
  test_state_transformation
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "unittest/python/${TEST}.py" "python")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})


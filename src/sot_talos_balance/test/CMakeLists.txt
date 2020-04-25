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

FOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
  INSTALL(FILES ${python} DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/test)
ENDFOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})

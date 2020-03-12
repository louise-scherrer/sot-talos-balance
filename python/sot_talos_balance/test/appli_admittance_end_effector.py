# flake8: noqa
import math

import numpy as np
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish, RosSubscribe
from dynamic_graph.sot.core import SOT, FeaturePosture, GainAdaptive, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.tracer_real_time import TracerRealTime

import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
import sot_talos_balance.talos.control_manager_conf as controlManagerConfig
import sot_talos_balance.talos.ft_wrist_calibration_conf as forceConf
import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *

robot.timeStep = robot.device.getTimeStep()

# --- EXPERIMENTAL SET UP ------------------------------------------------------

device = 'simu'
endEffector = 'rightWrist'
endEffectorWeight = forceConf.handWeight[device]
rightOC = forceConf.rightLeverArm
leftOC = forceConf.leftLeverArm

# --- SET INITIAL CONFIGURATION ------------------------------------------------

q = [0., 0., 1.018213, 0., 0., 0.]  # Base
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Left Leg
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Right Leg
q += [0.0, 0.006761]  # Chest
q += [0.25847, 0.173046, -0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Left Arm
# q += [-0.25847, -0.173046, 0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Right Arm
# q += [-0.25847, -0.0, 0.19, -1.61, 0., 0., 0.1, -0.005]             # Right Arm
q += [-0.0, -0.01, 0.00, -1.58, -0.01, 0., 0., -0.005]  # Right Arm
q += [0., 0.]  # Head
robot.device.set(q)

# --- CREATE ENTITIES ----------------------------------------------------------

robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)
robot.device_filters = create_device_filters(robot, robot.timeStep)
robot.imu_filters = create_imu_filters(robot, robot.timeStep)
robot.baseEstimator = create_base_estimator(robot, robot.timeStep, baseEstimatorConf)

# Get configuration vector
robot.e2q = EulerToQuat("e2q")
plug(robot.baseEstimator.q, robot.e2q.euler)

robot.forceCalibrator = create_ft_wrist_calibrator(robot, endEffectorWeight, rightOC, leftOC)
robot.controller = create_end_effector_admittance_controller(robot, endEffector, "EEAdmittance")

robot.controlManager = create_ctrl_manager(controlManagerConfig, robot.timeStep)
robot.controlManager.addCtrlMode('sot_input')
robot.controlManager.setCtrlMode('all', 'sot_input')

# --- HAND TASK ----------------------------------------------------------------

taskRightHand = MetaTaskKine6d('rh', robot.dynamic, 'rh', 'arm_right_7_joint')
handMgrip = np.eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRightHand.opmodif = matrixToTuple(handMgrip)
taskRightHand.feature.frame('desired')
taskRightHand.feature.selec.value = '111111'
taskRightHand.task.setWithDerivative(True)
taskRightHand.task.controlGain.value = 0
taskRightHand.feature.position.value = np.eye(4)
taskRightHand.feature.velocity.value = [0., 0., 0., 0., 0., 0.]
taskRightHand.featureDes.position.value = np.eye(4)
plug(robot.controller.dq, taskRightHand.featureDes.velocity)

# --- BASE TASK ----------------------------------------------------------------

taskWaist = MetaTaskKine6d('taskWaist', robot.dynamic, 'WT', robot.OperationalPointsMap['waist'])
taskWaist.feature.frame('desired')
taskWaist.gain.setConstant(300)
taskWaist.keep()
taskWaist.feature.selec.value = '111111'
locals()['taskWaist'] = taskWaist

# --- POSTURE TASK -------------------------------------------------------------

robot.taskPosture = Task('task_posture')
robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.feature = FeaturePosture('feature_posture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

robot.taskPosture.feature.selectDof(6, True)
robot.taskPosture.feature.selectDof(7, True)
robot.taskPosture.feature.selectDof(8, True)
robot.taskPosture.feature.selectDof(9, True)
robot.taskPosture.feature.selectDof(10, True)
robot.taskPosture.feature.selectDof(11, True)
robot.taskPosture.feature.selectDof(12, True)
robot.taskPosture.feature.selectDof(13, True)
robot.taskPosture.feature.selectDof(14, True)
robot.taskPosture.feature.selectDof(15, True)
robot.taskPosture.feature.selectDof(16, True)
robot.taskPosture.feature.selectDof(17, True)
robot.taskPosture.feature.selectDof(18, True)
robot.taskPosture.feature.selectDof(19, True)

robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# Plug SOT control to device through control manager
plug(robot.sot.control, robot.controlManager.ctrl_sot_input)
plug(robot.controlManager.u_safe, robot.device.control)
# plug(robot.sot.control, robot.device.control)

# --- PUSH THE TASKS -----------------------------------------------------------

robot.sot.push(robot.taskPosture.name)
robot.sot.push(taskRightHand.task.name)
robot.sot.push(taskWaist.task.name)

# # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.controller, 'w_force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_forceDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.forceCalibrator, 'leftWristForceOut', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.forceCalibrator, 'rightWristForceOut', robot=robot, data_type='vector')  # calibrated right wrench

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("end_effector_subscriber")
robot.subscriber.add("vector", "w_force", "/sot/controller/w_force")
robot.subscriber.add("vector", "force", "/sot/controller/force")
robot.subscriber.add("vector", "dq", "/sot/controller/dq")
robot.subscriber.add("vector", "w_dq", "/sot/controller/w_dq")
robot.subscriber.add("vector", "w_forceDes", "/sot/controller/w_forceDes")
robot.subscriber.add("vector", "leftWristForceOut", "/sot/forceCalibrator/leftWristForceOut")
robot.subscriber.add("vector", "rightWristForceOut", "/sot/forceCalibrator/rightWristForceOut")

# --- TRACER  ------------------------------------------------------------------

robot.tracer = TracerRealTime("force_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.controller, 'force')
addTrace(robot.tracer, robot.controller, 'w_force')
addTrace(robot.tracer, robot.controller, 'w_dq')
addTrace(robot.tracer, robot.controller, 'dq')

robot.tracer.start()

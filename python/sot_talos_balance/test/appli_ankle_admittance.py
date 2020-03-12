# flake8: noqa
from math import sqrt

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, Derivator_of_Vector, FeaturePosture, Task, operator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from dynamic_graph.tracer_real_time import TracerRealTime

import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
import sot_talos_balance.talos.control_manager_conf as controlManagerConf
import sot_talos_balance.talos.ft_calibration_conf as forceConf
import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Pendulum parameters
robotName = 'robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g / h)

# --- Parameter server
robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT', robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)

# -------------------------- DESIRED TRAJECTORY --------------------------

# --- Trajectory generators
robot.comTrajGen = create_com_trajectory_generator(dt, robot)

robot.lfTrajGen = create_pose_rpy_trajectory_generator(dt, robot, 'LF')
robot.lfToMatrix = PoseRollPitchYawToMatrixHomo('lf2m')
plug(robot.lfTrajGen.x, robot.lfToMatrix.sin)

robot.rfTrajGen = create_pose_rpy_trajectory_generator(dt, robot, 'RF')
robot.rfToMatrix = PoseRollPitchYawToMatrixHomo('rf2m')
plug(robot.rfTrajGen.x, robot.rfToMatrix.sin)

# --- Interface with controller entities
wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega
wp.waist.value = robot.dynamic.WT.value  # waist receives a full homogeneous matrix, but only the rotational part is controlled
plug(robot.lfToMatrix.sout, wp.footLeft)
plug(robot.rfToMatrix.sout, wp.footRight)
plug(robot.comTrajGen.x, wp.com)
plug(robot.comTrajGen.dx, wp.vcom)
plug(robot.comTrajGen.ddx, wp.acom)
robot.wp = wp

# --- Compute the values to use them in initialization
robot.wp.comDes.recompute(0)
robot.wp.dcmDes.recompute(0)
robot.wp.zmpDes.recompute(0)

# -------------------------- ESTIMATION --------------------------

# --- Base Estimation
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.baseEstimator = create_base_estimator(robot, dt, baseEstimatorConf)

# --- Reference frame
rf = SimpleReferenceFrame('rf')
rf.init(robotName)
plug(robot.dynamic.LF, rf.footLeft)
plug(robot.dynamic.RF, rf.footRight)
robot.rf = rf

# --- State transformation
stf = StateTransformation("stf")
stf.init()
plug(robot.rf.referenceFrame, stf.referenceFrame)
plug(robot.baseEstimator.q, stf.q_in)
plug(robot.baseEstimator.v, stf.v_in)
robot.stf = stf

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.stf.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.stf.q, robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0] * robotDim
robot.rdynamic.acceleration.value = [0.0] * robotDim

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robotName)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.stf.v, cdc_estimator.v)
robot.cdc_estimator = cdc_estimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
estimator.omega.value = omega
estimator.mass.value = 1.0
plug(robot.cdc_estimator.c, estimator.com)
plug(robot.cdc_estimator.dc, estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- Force calibration
robot.ftc = create_ft_calibrator(robot, forceConf)

# --- ZMP estimation
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF', 'left_sole_link')
robot.rdynamic.createOpPoint('sole_RF', 'right_sole_link')
plug(robot.rdynamic.sole_LF, zmp_estimator.poseLeft)
plug(robot.rdynamic.sole_RF, zmp_estimator.poseRight)
plug(robot.ftc.left_foot_force_out, zmp_estimator.wrenchLeft)
plug(robot.ftc.right_foot_force_out, zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- CoM control
Kp_com = [0.] * 2 + [4.]

comErr = operator.Substract_of_vector('comErr')
plug(robot.wp.comDes, comErr.sin1)
plug(robot.cdc_estimator.c, comErr.sin2)
robot.comErr = comErr

comControl = operator.Multiply_of_vector('comControl')
comControl.sin0.value = Kp_com
plug(robot.comErr.sout, comControl.sin1)
robot.comControl = comControl

forceControl = operator.Add_of_vector('forceControl')
plug(robot.comControl.sout, forceControl.sin1)
forceControl.sin2.value = [0.0, 0.0, mass * g]
robot.forceControl = forceControl

wrenchControl = operator.Mix_of_vector('wrenchControl')
wrenchControl.setSignalNumber(3)
wrenchControl.addSelec(1, 0, 3)
wrenchControl.addSelec(2, 3, 3)
wrenchControl.default.value = [0.0] * 6
plug(robot.forceControl.sout, wrenchControl.signal("sin1"))
wrenchControl.signal("sin2").value = [0.0] * 3
robot.wrenchControl = wrenchControl

# --- Distribute wrench
distribute = DistributeWrench('distribute')
plug(robot.e2q.quaternion, distribute.q)
plug(robot.wrenchControl.sout, distribute.wrenchDes)
distribute.init(robotName)
robot.wrenchDistributor = distribute

# --- Left foot
robot.leftAnkleController = create_ankle_admittance_controller([0, 0], robot, "left", "leftController")

# --- Right foot
robot.rightAnkleController = create_ankle_admittance_controller([0, 0], robot, "right", "rightController")

# --- Control Manager
robot.controlManager = create_ctrl_manager(controlManagerConf, dt, robot_name='robot')
robot.controlManager.addCtrlMode('sot_input')
robot.controlManager.setCtrlMode('all', 'sot_input')
robot.controlManager.addEmergencyStopSIN('zmp')

# -------------------------- SOT CONTROL --------------------------

# --- Upper body
robot.taskUpperBody = Task('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

# robotDim = robot.dynamic.getDimension() # 38
robot.taskUpperBody.feature.selectDof(18, True)
robot.taskUpperBody.feature.selectDof(19, True)
robot.taskUpperBody.feature.selectDof(20, True)
robot.taskUpperBody.feature.selectDof(21, True)
robot.taskUpperBody.feature.selectDof(22, True)
robot.taskUpperBody.feature.selectDof(23, True)
robot.taskUpperBody.feature.selectDof(24, True)
robot.taskUpperBody.feature.selectDof(25, True)
robot.taskUpperBody.feature.selectDof(26, True)
robot.taskUpperBody.feature.selectDof(27, True)
robot.taskUpperBody.feature.selectDof(28, True)
robot.taskUpperBody.feature.selectDof(29, True)
robot.taskUpperBody.feature.selectDof(30, True)
robot.taskUpperBody.feature.selectDof(31, True)
robot.taskUpperBody.feature.selectDof(32, True)
robot.taskUpperBody.feature.selectDof(33, True)
robot.taskUpperBody.feature.selectDof(34, True)
robot.taskUpperBody.feature.selectDof(35, True)
robot.taskUpperBody.feature.selectDof(36, True)
robot.taskUpperBody.feature.selectDof(37, True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- RIGHT ANKLE PITCH
RightPitchJoint = 16

rightPitchSelec = Selec_of_vector("rightPitchSelec")
rightPitchSelec.selec(1, 2)
plug(robot.rightAnkleController.dRP, rightPitchSelec.sin)

robot.rightAnklePitchTask = MetaTaskKineJoint(robot.dynamic, RightPitchJoint)
robot.rightAnklePitchTask.featureDes.errorIN.value = [0.0]
robot.rightAnklePitchTask.task.controlGain.value = 0
robot.rightAnklePitchTask.task.setWithDerivative(True)
plug(rightPitchSelec.sout, robot.rightAnklePitchTask.featureDes.errordotIN)

# --- RIGHT ANKLE ROLL
RightRollJoint = 17

rightRollSelec = Selec_of_vector("rightRollSelec")
rightRollSelec.selec(0, 1)
plug(robot.rightAnkleController.dRP, rightRollSelec.sin)

robot.rightAnkleRollTask = MetaTaskKineJoint(robot.dynamic, RightRollJoint)
robot.rightAnkleRollTask.featureDes.errorIN.value = [0.0]
robot.rightAnkleRollTask.task.controlGain.value = 0
robot.rightAnkleRollTask.task.setWithDerivative(True)
plug(rightRollSelec.sout, robot.rightAnkleRollTask.featureDes.errordotIN)

# --- LEFT ANKLE PITCH
LeftPitchJoint = 10

leftPitchSelec = Selec_of_vector("leftPitchSelec")
leftPitchSelec.selec(1, 2)
plug(robot.leftAnkleController.dRP, leftPitchSelec.sin)

robot.leftAnklePitchTask = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint)
robot.leftAnklePitchTask.featureDes.errorIN.value = [0.0]
robot.leftAnklePitchTask.task.controlGain.value = 0
robot.leftAnklePitchTask.task.setWithDerivative(True)
plug(leftPitchSelec.sout, robot.leftAnklePitchTask.featureDes.errordotIN)

# --- LEFT ANKLE ROLL
LeftRollJoint = 11

leftRollSelec = Selec_of_vector("leftRollSelec")
leftRollSelec.selec(0, 1)
plug(robot.leftAnkleController.dRP, leftRollSelec.sin)

robot.leftAnkleRollTask = MetaTaskKineJoint(robot.dynamic, LeftRollJoint)
robot.leftAnkleRollTask.featureDes.errorIN.value = [0.0]
robot.leftAnkleRollTask.task.controlGain.value = 0
robot.leftAnkleRollTask.task.setWithDerivative(True)
plug(leftRollSelec.sout, robot.leftAnkleRollTask.featureDes.errordotIN)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(100)
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(100)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.wp.comDes, robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 10
robot.taskCom.feature.selec.value = '011'  # needed ?

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist', robot.dynamic, 'WT', robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position)
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- Plug SOT control to device through control manager
plug(robot.sot.control, robot.controlManager.ctrl_sot_input)
plug(robot.controlManager.u_safe, robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)
# robot.sot.push(robot.taskPos.name)
# robot.device.control.recompute(0) # this crashes as it employs joint sensors which are not ready yet

# --- Fix robot.dynamic inputs
plug(robot.device.velocity, robot.dynamic.velocity)
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.device.velocity, robot.dvdt.sin)
plug(robot.dvdt.sout, robot.dynamic.acceleration)

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')

create_topic(robot.publisher, robot.wp, 'comDes', robot=robot, data_type='vector')  # desired CoM

create_topic(robot.publisher, robot.cdc_estimator, 'c', robot=robot, data_type='vector')  # estimated CoM
create_topic(robot.publisher, robot.cdc_estimator, 'dc', robot=robot, data_type='vector')  # estimated CoM velocity

create_topic(robot.publisher, robot.dynamic, 'com', robot=robot, data_type='vector')  # resulting SOT CoM

create_topic(robot.publisher, robot.estimator, 'dcm', robot=robot, data_type='vector')  # estimated DCM

create_topic(robot.publisher, robot.dynamic, 'zmp', robot=robot, data_type='vector')  # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot=robot, data_type='vector')  # estimated ZMP

#create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # measured left wrench
#create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')               # measured right wrench

#create_topic(robot.publisher, robot.device_filters.ft_LF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered left wrench
#create_topic(robot.publisher, robot.device_filters.ft_RF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered right wrench

create_topic(robot.publisher, robot.wrenchDistributor, 'wrenchLeft', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'wrenchRight', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot=robot, data_type='vector')  # calibrated right wrench

# --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.wp, 'comDes')  # desired CoM

addTrace(robot.tracer, robot.cdc_estimator, 'c')  # estimated CoM
addTrace(robot.tracer, robot.cdc_estimator, 'dc')  # estimated CoM velocity

addTrace(robot.tracer, robot.dynamic, 'com')  # resulting SOT CoM

addTrace(robot.tracer, robot.estimator, 'dcm')  # estimated DCM

addTrace(robot.tracer, robot.dynamic, 'zmp')  # SOT ZMP
addTrace(robot.tracer, robot.zmp_estimator, 'zmp')  # estimated ZMP

addTrace(robot.tracer, robot.ftc, 'left_foot_force_out')  # calibrated left wrench
addTrace(robot.tracer, robot.ftc, 'right_foot_force_out')  # calibrated right wrench

robot.tracer.start()

# flake8: noqa
from math import sqrt

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, Derivator_of_Vector, FeaturePosture, MatrixHomoToPoseQuaternion, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from dynamic_graph.tracer_real_time import TracerRealTime

import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import sot_talos_balance.talos.control_manager_conf as cm_conf
import sot_talos_balance.talos.ft_calibration_conf as ft_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.simple_controller_6d import SimpleController6d

cm_conf.CTRL_MAX = 1000.0  # temporary hack

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Pendulum parameters
robot_name = 'robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g / h)

# --- Parameter server
robot.param_server = create_parameter_server(param_server_conf, dt)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT', robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)

# -------------------------- DESIRED TRAJECTORY --------------------------

# --- Trajectory generators

# --- CoM
robot.comTrajGen = create_com_trajectory_generator(dt, robot)

# --- Left foot
robot.lfTrajGen = create_pose_rpy_trajectory_generator(dt, robot, 'LF')
# robot.lfTrajGen.x.recompute(0) # trigger computation of initial value
robot.lfToMatrix = PoseRollPitchYawToMatrixHomo('lf2m')
plug(robot.lfTrajGen.x, robot.lfToMatrix.sin)

# --- Right foot
robot.rfTrajGen = create_pose_rpy_trajectory_generator(dt, robot, 'RF')
# robot.rfTrajGen.x.recompute(0) # trigger computation of initial value
robot.rfToMatrix = PoseRollPitchYawToMatrixHomo('rf2m')
plug(robot.rfTrajGen.x, robot.rfToMatrix.sin)

# --- Waist
robot.waistTrajGen = create_orientation_rpy_trajectory_generator(dt, robot, 'WT')
# robot.waistTrajGen.x.recompute(0) # trigger computation of initial value

robot.waistMix = Mix_of_vector("waistMix")
robot.waistMix.setSignalNumber(3)
robot.waistMix.addSelec(1, 0, 3)
robot.waistMix.addSelec(2, 3, 3)
robot.waistMix.default.value = [0.0] * 6
robot.waistMix.signal("sin1").value = [0.0] * 3
plug(robot.waistTrajGen.x, robot.waistMix.signal("sin2"))

robot.waistToMatrix = PoseRollPitchYawToMatrixHomo('w2m')
plug(robot.waistMix.sout, robot.waistToMatrix.sin)

# --- Rho
robot.rhoTrajGen = create_scalar_trajectory_generator(dt, 0.5, 'rhoTrajGen')
robot.rhoScalar = Component_of_vector("rho_scalar")
robot.rhoScalar.setIndex(0)
plug(robot.rhoTrajGen.x, robot.rhoScalar.sin)

# --- Interface with controller entities

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega
plug(robot.waistToMatrix.sout, wp.waist)
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
robot.base_estimator = create_base_estimator(robot, dt, base_estimator_conf)

robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.LF, robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.RF, robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# robot.be_filters              = create_be_filters(robot, dt)

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
#plug(robot.base_estimator.q,robot.rdynamic.position)

robot.baseselec = Selec_of_vector("base_selec")
robot.baseselec.selec(0, 6)
plug(robot.base_estimator.q, robot.baseselec.sin)
plug(robot.baseselec.sout, robot.rdynamic.ffposition)

plug(robot.device.state, robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0] * robotDim
robot.rdynamic.acceleration.value = [0.0] * robotDim

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.base_estimator.v, cdc_estimator.v)
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
robot.ftc = create_ft_calibrator(robot, ft_conf)

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

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all', 'sot_input')

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

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.rdynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(1.)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position)  #.errorIN?
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.rdynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(1.)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position)  #.errorIN?
locals()['contactRF'] = robot.contactRF

## --- COM height
#robot.taskComH = MetaTaskKineCom(robot.dynamic, name='comH')
#plug(robot.wp.comDes,robot.taskComH.featureDes.errorIN)
#robot.taskComH.task.controlGain.value = 100.
#robot.taskComH.feature.selec.value = '100'

# --- COM
robot.taskCom = MetaTaskKineCom(robot.rdynamic)
plug(robot.wp.comDes, robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 1.
#robot.taskCom.feature.selec.value = '011'

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist', robot.rdynamic, 'WT', robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(1.)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position)
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- Plug SOT control to device through control manager
plug(robot.sot.control, robot.cm.ctrl_sot_input)
plug(robot.cm.u_safe, robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
#robot.sot.push(robot.taskComH.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)
#robot.sot.push(robot.taskWaist.name)

# --- Fix robot.dynamic inputs
#robot.baseselec = Selec_of_vector('baseselec')
#robot.baseselec.selec(0, 6)
#plug(robot.base_estimator.q, robot.baseselec.sin)
#plug(robot.baseselec.sout, robot.dynamic.ffposition)

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

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot=robot, data_type='vector')  # calibrated right wrench

create_topic(robot.publisher, robot.zmp_estimator, 'copRight', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.zmp_estimator, 'copLeft', robot=robot, data_type='vector')

## --- TRACER
#robot.tracer = TracerRealTime("com_tracer")
#robot.tracer.setBufferSize(80*(2**20))
#robot.tracer.open('/tmp','dg_','.dat')
#robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

#addTrace(robot.tracer, robot.wp, 'comDes')                      # desired CoM

#addTrace(robot.tracer, robot.cdc_estimator, 'c')                # estimated CoM
#addTrace(robot.tracer, robot.cdc_estimator, 'dc')               # estimated CoM velocity

#addTrace(robot.tracer, robot.dynamic, 'com')                    # resulting SOT CoM

#addTrace(robot.tracer, robot.estimator, 'dcm')                  # estimated DCM

#addTrace(robot.tracer, robot.dynamic, 'zmp')                    # SOT ZMP
#addTrace(robot.tracer, robot.zmp_estimator, 'zmp')              # estimated ZMP

#addTrace(robot.tracer, robot.ftc, 'left_foot_force_out')        # calibrated left wrench
#addTrace(robot.tracer,  robot.ftc, 'right_foot_force_out')      # calibrated right wrench

#robot.tracer.start()

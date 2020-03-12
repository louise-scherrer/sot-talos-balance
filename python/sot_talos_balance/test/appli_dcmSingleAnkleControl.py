# flake8: noqa
from math import sqrt

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, Derivator_of_Vector, FeaturePosture, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.operator import Multiply_double_vector
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from dynamic_graph.tracer_real_time import TracerRealTime

import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import sot_talos_balance.talos.control_manager_conf as cm_conf
import sot_talos_balance.talos.ft_calibration_conf as ft_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint

cm_conf.CTRL_MAX = 10.0  # temporary hack

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

# --- Phase
robot.phaseTrajGen = create_scalar_trajectory_generator(dt, 0.5, 'phaseTrajGen')
robot.phaseScalar = Component_of_vector("phase_scalar")
robot.phaseScalar.setIndex(0)
plug(robot.phaseTrajGen.x, robot.phaseScalar.sin)

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
# robot.be_filters              = create_be_filters(robot, dt)

# --- Reference frame

rf = SimpleReferenceFrame('rf')
rf.init(robot_name)
plug(robot.dynamic.LF, rf.footLeft)
plug(robot.dynamic.RF, rf.footRight)
robot.rf = rf

# --- State transformation
stf = StateTransformation("stf")
stf.init()
plug(robot.rf.referenceFrame, stf.referenceFrame)
plug(robot.base_estimator.q, stf.q_in)
plug(robot.base_estimator.v, stf.v_in)
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
cdc_estimator.init(dt, robot_name)
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

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [5.0, 5.0, 5.0]
Ki_dcm = [0.0, 0.0, 0.0]  # zero (to be set later)
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
dcm_controller.omega.value = omega

plug(robot.cdc_estimator.c, dcm_controller.com)
plug(robot.estimator.dcm, dcm_controller.dcm)

plug(robot.wp.zmpDes, dcm_controller.zmpDes)
plug(robot.wp.dcmDes, dcm_controller.dcmDes)

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [1.0, 1.0, 1.0]  # this value is employed later

# --- Distribute wrench
distribute = create_simple_distribute_wrench()
plug(robot.e2q.quaternion, distribute.q)
plug(robot.dcm_control.wrenchRef, distribute.wrenchDes)
plug(robot.rhoScalar.sout, distribute.rho)
distribute.init(robot_name)
robot.distribute = distribute

# --- Ankle controllers
LeftPitchJoint = 4
LeftRollJoint = 5
RightPitchJoint = 10
RightRollJoint = 11

KpPitch = [1e-3]
KpRoll = [1e-3]

# --- RIGHT ANKLE PITCH
controller = SimpleAdmittanceController("rightPitchAnkleController")
controller.Kp.value = [0.0]

robot.stateselecRP = Selec_of_vector("stateselecRP")
robot.stateselecRP.selec(RightPitchJoint + 6, RightPitchJoint + 7)
plug(robot.device.state, robot.stateselecRP.sin)
plug(robot.stateselecRP.sout, controller.state)

robot.tauselecRP = Selec_of_vector("tauselecRP")
robot.tauselecRP.selec(RightPitchJoint, RightPitchJoint + 1)
plug(robot.device.ptorque, robot.tauselecRP.sin)
plug(robot.tauselecRP.sout, controller.tau)

robot.rightPitchSelec = Selec_of_vector("rightPitchSelec")
robot.rightPitchSelec.selec(4, 5)
plug(robot.distribute.ankleWrenchRight, robot.rightPitchSelec.sin)

robot.multRP = Multiply_double_vector("multRP")
robot.multRP.sin1.value = -1.0
plug(robot.rightPitchSelec.sout, robot.multRP.sin2)
plug(robot.multRP.sout, controller.tauDes)
#plug(robot.rightPitchSelec.sout, controller.tauDes)

controller.init(dt, 1)
controller.setPosition([robot.device.state.value[RightPitchJoint + 6]])
robot.rightPitchAnkleController = controller

robot.rightAnklePitchTask = MetaTaskKineJoint(robot.dynamic, RightPitchJoint + 6)
robot.rightAnklePitchTask.task.controlGain.value = 0
robot.rightAnklePitchTask.task.setWithDerivative(True)
plug(robot.rightPitchAnkleController.qRef, robot.rightAnklePitchTask.featureDes.errorIN)
plug(robot.rightPitchAnkleController.dqRef, robot.rightAnklePitchTask.featureDes.errordotIN)

# --- RIGHT ANKLE ROLL
controller = SimpleAdmittanceController("rightRollAnkleController")
controller.Kp.value = [0.0]

robot.stateselecRR = Selec_of_vector("stateselecRR")
robot.stateselecRR.selec(RightRollJoint + 6, RightRollJoint + 7)
plug(robot.device.state, robot.stateselecRR.sin)
plug(robot.stateselecRR.sout, controller.state)

robot.tauselecRR = Selec_of_vector("tauselecRR")
robot.tauselecRR.selec(RightRollJoint, RightRollJoint + 1)
plug(robot.device.ptorque, robot.tauselecRR.sin)
plug(robot.tauselecRR.sout, controller.tau)

robot.rightRollSelec = Selec_of_vector("rightRollSelec")
robot.rightRollSelec.selec(3, 4)
plug(robot.distribute.ankleWrenchRight, robot.rightRollSelec.sin)

robot.multRR = Multiply_double_vector("multRR")
robot.multRR.sin1.value = -1.0
plug(robot.rightRollSelec.sout, robot.multRR.sin2)
plug(robot.multRR.sout, controller.tauDes)
#plug(robot.rightRollSelec.sout, controller.tauDes)

controller.init(dt, 1)
controller.setPosition([robot.device.state.value[RightRollJoint + 6]])
robot.rightRollAnkleController = controller

robot.rightAnkleRollTask = MetaTaskKineJoint(robot.dynamic, RightRollJoint + 6)
robot.rightAnkleRollTask.task.controlGain.value = 0
robot.rightAnkleRollTask.task.setWithDerivative(True)
plug(robot.rightRollAnkleController.qRef, robot.rightAnkleRollTask.featureDes.errorIN)
plug(robot.rightRollAnkleController.dqRef, robot.rightAnkleRollTask.featureDes.errordotIN)

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all', 'sot_input')
robot.cm.addEmergencyStopSIN('zmp')
robot.cm.addEmergencyStopSIN('distribute')

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
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position)  #.errorIN?
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position)  #.errorIN?
#robot.contactRF.feature.selec.value = '000111'
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.wp.comDes, robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 10
robot.taskCom.feature.selec.value = '100'

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
plug(robot.sot.control, robot.cm.ctrl_sot_input)
plug(robot.cm.u_safe, robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.rightAnklePitchTask.task.name)
robot.sot.push(robot.rightAnkleRollTask.task.name)
robot.sot.push(robot.keepWaist.task.name)

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

create_topic(robot.publisher, robot.dcm_control, 'dcmDes', robot=robot, data_type='vector')  # desired DCM
create_topic(robot.publisher, robot.estimator, 'dcm', robot=robot, data_type='vector')  # estimated DCM

create_topic(robot.publisher, robot.dcm_control, 'zmpDes', robot=robot, data_type='vector')  # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', robot=robot, data_type='vector')  # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot=robot, data_type='vector')  # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot=robot, data_type='vector')  # reference ZMP

create_topic(robot.publisher, robot.dcm_control, 'wrenchRef', robot=robot, data_type='vector')  # unoptimized reference wrench
create_topic(robot.publisher, robot.distribute, 'wrenchLeft', robot=robot, data_type='vector')  # reference left wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRight', robot=robot, data_type='vector')  # reference right wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRef', robot=robot, data_type='vector')  # optimized reference wrench

#create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # measured left wrench
#create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')               # measured right wrench

#create_topic(robot.publisher, robot.device_filters.ft_LF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered left wrench
#create_topic(robot.publisher, robot.device_filters.ft_RF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered right wrench

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot=robot, data_type='vector')  # calibrated right wrench

create_topic(robot.publisher, robot.rightPitchAnkleController, 'tau', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightPitchAnkleController, 'tauDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightPitchAnkleController, 'qRef', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightPitchAnkleController, 'dqRef', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.rightRollAnkleController, 'tau', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollAnkleController, 'tauDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollAnkleController, 'qRef', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollAnkleController, 'dqRef', robot=robot, data_type='vector')

## --- TRACER
#robot.tracer = TracerRealTime("com_tracer")
#robot.tracer.setBufferSize(80*(2**20))
#robot.tracer.open('/tmp','dg_','.dat')
#robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

#addTrace(robot.tracer, robot.wp, 'comDes')                      # desired CoM

#addTrace(robot.tracer, robot.cdc_estimator, 'c')                # estimated CoM
#addTrace(robot.tracer, robot.cdc_estimator, 'dc')               # estimated CoM velocity

#addTrace(robot.tracer, robot.com_admittance_control, 'comRef')  # reference CoM
#addTrace(robot.tracer, robot.dynamic, 'com')                    # resulting SOT CoM

#addTrace(robot.tracer, robot.dcm_control, 'dcmDes')             # desired DCM
#addTrace(robot.tracer, robot.estimator, 'dcm')                  # estimated DCM

#addTrace(robot.tracer, robot.dcm_control, 'zmpDes')             # desired ZMP
#addTrace(robot.tracer, robot.dynamic, 'zmp')                    # SOT ZMP
#addTrace(robot.tracer, robot.zmp_estimator, 'zmp')              # estimated ZMP
#addTrace(robot.tracer, robot.dcm_control, 'zmpRef')             # reference ZMP

#addTrace(robot.tracer, robot.dcm_control, 'wrenchRef')          # unoptimized reference wrench
#addTrace(robot.tracer, robot.distribute, 'wrenchLeft')          # reference left wrench
#addTrace(robot.tracer, robot.distribute, 'wrenchRight')         # reference right wrench
#addTrace(robot.tracer, robot.distribute, 'wrenchRef')           # optimized reference wrench

#addTrace(robot.tracer, robot.ftc, 'left_foot_force_out')        # calibrated left wrench
#addTrace(robot.tracer,  robot.ftc, 'right_foot_force_out')      # calibrated right wrench

#robot.tracer.start()

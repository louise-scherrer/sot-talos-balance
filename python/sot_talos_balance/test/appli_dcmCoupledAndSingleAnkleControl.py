# flake8: noqa
from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.coupled_admittance_controller import CoupledAdmittanceController
from sot_talos_balance.saturation import Saturation
import sot_talos_balance.talos.parameter_server_conf   as param_server_conf
import sot_talos_balance.talos.control_manager_conf    as cm_conf
import sot_talos_balance.talos.base_estimator_conf     as base_estimator_conf
import sot_talos_balance.talos.ft_calibration_conf     as ft_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.operator import Multiply_double_vector
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from math import sqrt, pi
import numpy as np

from dynamic_graph.tracer_real_time import TracerRealTime

from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

cm_conf.CTRL_MAX = 1000.0 # temporary hack

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Pendulum parameters
robot_name='robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

# --- Parameter server
robot.param_server = create_parameter_server(param_server_conf,dt)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF',robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF',robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT',robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)

# --- Ankle velocity saturation
kSat = 10.0
qLimSat = [pi/6]
dqLimSat = [0.3]

# -------------------------- DESIRED TRAJECTORY --------------------------

# --- Trajectory generators

# --- CoM
robot.comTrajGen = create_com_trajectory_generator(dt, robot)

# --- Left foot
robot.lfTrajGen  = create_pose_rpy_trajectory_generator(dt, robot, 'LF')
# robot.lfTrajGen.x.recompute(0) # trigger computation of initial value
robot.lfToMatrix = PoseRollPitchYawToMatrixHomo('lf2m')
plug(robot.lfTrajGen.x, robot.lfToMatrix.sin)

# --- Right foot
robot.rfTrajGen  = create_pose_rpy_trajectory_generator(dt, robot, 'RF')
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
robot.waistMix.default.value = [0.0]*6
robot.waistMix.signal("sin1").value = [0.0]*3
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
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf)
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
plug(robot.rf.referenceFrame,stf.referenceFrame)
plug(robot.base_estimator.q,stf.q_in)
plug(robot.base_estimator.v,stf.v_in)
robot.stf = stf

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.stf.q,e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.stf.q,robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0]*robotDim
robot.rdynamic.acceleration.value = [0.0]*robotDim

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
plug(robot.cdc_estimator.dc,estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- Force calibration
robot.ftc = create_ft_calibrator(robot,ft_conf)

# --- ZMP estimation
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF','left_sole_link')
robot.rdynamic.createOpPoint('sole_RF','right_sole_link')
plug(robot.rdynamic.sole_LF,zmp_estimator.poseLeft)
plug(robot.rdynamic.sole_RF,zmp_estimator.poseRight)
plug(robot.ftc.left_foot_force_out,zmp_estimator.wrenchLeft)
plug(robot.ftc.right_foot_force_out,zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [5.0,5.0,5.0]
Ki_dcm = [0.0,0.0,0.0] # zero (to be set later)
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
dcm_controller.omega.value = omega

plug(robot.cdc_estimator.c,dcm_controller.com)
plug(robot.estimator.dcm,dcm_controller.dcm)

plug(robot.wp.zmpDes, dcm_controller.zmpDes)
plug(robot.wp.dcmDes, dcm_controller.dcmDes)

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [1.0,1.0,1.0] # this value is employed later

# --- Int identity
robot.phaseSwitch = IntIdentity("phaseSwitch")
robot.phaseSwitch.sin.value = 0

# --- Distribute wrench
distribute = create_simple_distribute_wrench()
plug(robot.e2q.quaternion, distribute.q)
plug(robot.dcm_control.wrenchRef, distribute.wrenchDes)
plug(robot.rhoScalar.sout, distribute.rho)
plug(robot.phaseSwitch.sout, distribute.phase)
distribute.init(robot_name)
robot.distribute = distribute

# ---  Ankle admittance controllers
LeftPitchJoint = 4
LeftRollJoint = 5
RightPitchJoint = 10
RightRollJoint = 11

kSumPitch = [1e-3]
kSumRoll = [0.0]
kDiffPitch = [1e-6]
kDiffRoll = [0.0]

# --- Pitch
controller = CoupledAdmittanceController("pitchController")
controller.kSum.value = kSumPitch
controller.kDiff.value = kDiffPitch

# Right pitch
robot.tauRP = Selec_of_vector("tauRP")
robot.tauRP.selec(RightPitchJoint, RightPitchJoint+1)
plug(robot.device.ptorque, robot.tauRP.sin)
plug(robot.tauRP.sout, controller.tauR)

robot.tauDesRP = Selec_of_vector("tauDesRP")
robot.tauDesRP.selec(4, 5)
plug(robot.distribute.ankleWrenchRight, robot.tauDesRP.sin)

robot.multRP = Multiply_double_vector("multRP")
robot.multRP.sin1.value = -1.0
plug(robot.tauDesRP.sout, robot.multRP.sin2)
plug(robot.multRP.sout, controller.tauDesR)

# Left pitch
robot.tauLP = Selec_of_vector("tauLP")
robot.tauLP.selec(LeftPitchJoint, LeftPitchJoint+1)
plug(robot.device.ptorque, robot.tauLP.sin)
plug(robot.tauLP.sout, controller.tauL)

robot.tauDesLP = Selec_of_vector("tauDesLP")
robot.tauDesLP.selec(4, 5)
plug(robot.distribute.ankleWrenchLeft, robot.tauDesLP.sin)

robot.multLP = Multiply_double_vector("multLP")
robot.multLP.sin1.value = -1.0
plug(robot.tauDesLP.sout, robot.multLP.sin2)
plug(robot.multLP.sout, controller.tauDesL)

robot.pitchController = controller

# --- Roll
controller = CoupledAdmittanceController("rollController")
controller.kSum.value = kSumRoll
controller.kDiff.value = kDiffRoll

# Right roll
robot.tauRR = Selec_of_vector("tauRR")
robot.tauRR.selec(RightRollJoint, RightRollJoint+1)
plug(robot.device.ptorque, robot.tauRR.sin)
plug(robot.tauRR.sout, controller.tauR)

robot.tauDesRR = Selec_of_vector("tauDesRR")
robot.tauDesRR.selec(3, 4)
plug(robot.distribute.ankleWrenchRight, robot.tauDesRR.sin)

robot.multRR = Multiply_double_vector("multRR")
robot.multRR.sin1.value = -1.0
plug(robot.tauDesRR.sout, robot.multRR.sin2)
plug(robot.multRR.sout, controller.tauDesR)

# Left roll
robot.tauLR = Selec_of_vector("tauLR")
robot.tauLR.selec(LeftRollJoint, LeftRollJoint+1)
plug(robot.device.ptorque, robot.tauLR.sin)
plug(robot.tauLR.sout, controller.tauL)

robot.tauDesLR = Selec_of_vector("tauDesLR")
robot.tauDesLR.selec(3, 4)
plug(robot.distribute.ankleWrenchLeft, robot.tauDesLR.sin)

robot.multLR = Multiply_double_vector("multLR")
robot.multLR.sin1.value = -1.0
plug(robot.tauDesLR.sout, robot.multLR.sin2)
plug(robot.multLR.sout, controller.tauDesL)

robot.rollController = controller

# --- Decoupled controllers
KpPitch = [1e-3]
KpRoll = [1e-3]

# --- RIGHT ANKLE PITCH
robot.qRP = Selec_of_vector("qRP")
robot.qRP.selec(RightPitchJoint+6, RightPitchJoint+7)
plug(robot.device.state, robot.qRP.sin)

controller = SimpleAdmittanceController("rightPitchAnkleController")
controller.Kp.value = KpPitch
plug(robot.qRP.sout, controller.state)
plug(robot.tauRP.sout, controller.tau)
plug(robot.multRP.sout, controller.tauDes)
controller.init(dt, 1)
controller.setPosition([robot.device.state.value[RightPitchJoint+6]])
robot.rightPitchAnkleController = controller

# --- RIGHT ANKLE ROLL
robot.qRR = Selec_of_vector("qRR")
robot.qRR.selec(RightRollJoint+6, RightRollJoint+7)
plug(robot.device.state, robot.qRR.sin)

controller = SimpleAdmittanceController("rightRollAnkleController")
controller.Kp.value = KpRoll
plug(robot.qRR.sout, controller.state)
plug(robot.tauRR.sout, controller.tau)
plug(robot.multRR.sout, controller.tauDes)
controller.init(dt, 1)
controller.setPosition([robot.device.state.value[RightRollJoint+6]])
robot.rightRollAnkleController = controller

# --- LEFT ANKLE PITCH
robot.qLP = Selec_of_vector("qLP")
robot.qLP.selec(LeftPitchJoint+6, LeftPitchJoint+7)
plug(robot.device.state, robot.qLP.sin)

controller = SimpleAdmittanceController("leftPitchAnkleController")
controller.Kp.value = KpPitch
plug(robot.qLP.sout, controller.state)
plug(robot.tauLP.sout, controller.tau)
plug(robot.multLP.sout, controller.tauDes)
controller.init(dt, 1)
controller.setPosition([robot.device.state.value[LeftPitchJoint+6]])
robot.leftPitchAnkleController = controller

# --- LEFT ANKLE ROLL
robot.qLR = Selec_of_vector("qLR")
robot.qLR.selec(LeftRollJoint+6, LeftRollJoint+7)
plug(robot.device.state, robot.qLR.sin)

controller = SimpleAdmittanceController("leftRollAnkleController")
controller.Kp.value = KpRoll
plug(robot.qLR.sout, controller.state)
plug(robot.tauLR.sout, controller.tau)
plug(robot.multLR.sout, controller.tauDes)
controller.init(dt, 1)
controller.setPosition([robot.device.state.value[LeftRollJoint+6]])
robot.leftRollAnkleController = controller

# --- Ankle joint selector
robot.ajs = AnkleJointSelector('ajs')
robot.ajs.init(robot.dynamic.getDimension())

plug(robot.phaseSwitch.sout, robot.ajs.phase)

plug(robot.rightRollAnkleController.dqRef, robot.ajs.rightRollDecoupled)
plug(robot.rollController.dqRefR, robot.ajs.rightRollCoupled)

plug(robot.rightPitchAnkleController.dqRef, robot.ajs.rightPitchDecoupled)
plug(robot.pitchController.dqRefR, robot.ajs.rightPitchCoupled)

plug(robot.leftRollAnkleController.dqRef, robot.ajs.leftRollDecoupled)
plug(robot.rollController.dqRefL, robot.ajs.leftRollCoupled)

plug(robot.leftPitchAnkleController.dqRef, robot.ajs.leftPitchDecoupled)
plug(robot.pitchController.dqRefL, robot.ajs.leftPitchCoupled)

# --- ANKLE TASKS
robot.saturationRP = Saturation("saturationRP")
plug(robot.qRP.sout, robot.saturationRP.x)
plug(robot.ajs.rightPitch, robot.saturationRP.y)
robot.saturationRP.k.value = kSat
robot.saturationRP.xLim.value = qLimSat
robot.saturationRP.yLim.value = dqLimSat

robot.taskRP = MetaTaskKineJoint(robot.dynamic, RightPitchJoint+6)
robot.taskRP.task.controlGain.value = 0
robot.taskRP.task.setWithDerivative(True)
robot.taskRP.featureDes.errorIN.value = [0.0]
plug(robot.saturationRP.yOut, robot.taskRP.featureDes.errordotIN)
plug(robot.ajs.selecRight, robot.taskRP.task.controlSelec)

robot.saturationRR = Saturation("saturationRR")
plug(robot.qRR.sout, robot.saturationRR.x)
plug(robot.ajs.rightRoll, robot.saturationRR.y)
robot.saturationRR.k.value = kSat
robot.saturationRR.xLim.value = qLimSat
robot.saturationRR.yLim.value = dqLimSat

robot.taskRR = MetaTaskKineJoint(robot.dynamic, RightRollJoint+6)
robot.taskRR.task.controlGain.value = 0
robot.taskRR.task.setWithDerivative(True)
robot.taskRR.featureDes.errorIN.value = [0.0]
plug(robot.saturationRR.yOut, robot.taskRR.featureDes.errordotIN)
plug(robot.ajs.selecRight, robot.taskRR.task.controlSelec)

robot.saturationLP = Saturation("saturationLP")
plug(robot.qLP.sout, robot.saturationLP.x)
plug(robot.ajs.leftPitch, robot.saturationLP.y)
robot.saturationLP.k.value = kSat
robot.saturationLP.xLim.value = qLimSat
robot.saturationLP.yLim.value = dqLimSat

robot.taskLP = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint+6)
robot.taskLP.task.controlGain.value = 0
robot.taskLP.task.setWithDerivative(True)
robot.taskLP.featureDes.errorIN.value = [0.0]
plug(robot.saturationLP.yOut, robot.taskLP.featureDes.errordotIN)
plug(robot.ajs.selecLeft, robot.taskLP.task.controlSelec)

robot.saturationLR = Saturation("saturationLR")
plug(robot.qLR.sout, robot.saturationLR.x)
plug(robot.ajs.leftRoll, robot.saturationLR.y)
robot.saturationLR.k.value = kSat
robot.saturationLR.xLim.value = qLimSat
robot.saturationLR.yLim.value = dqLimSat

robot.taskLR = MetaTaskKineJoint(robot.dynamic, LeftRollJoint+6)
robot.taskLR.task.controlGain.value = 0
robot.taskLR.task.setWithDerivative(True)
robot.taskLR.featureDes.errorIN.value = [0.0]
plug(robot.saturationLR.yOut, robot.taskLR.featureDes.errordotIN)
plug(robot.ajs.selecLeft, robot.taskLR.task.controlSelec)

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all','sot_input')
robot.cm.addEmergencyStopSIN('zmp')
robot.cm.addEmergencyStopSIN('distribute')

# -------------------------- SOT CONTROL --------------------------
# --- UPPER BODY POSTURE
robot.taskUpperBody = Task ('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

robot.taskUpperBody.feature.selectDof(18,True)
robot.taskUpperBody.feature.selectDof(19,True)
robot.taskUpperBody.feature.selectDof(20,True)
robot.taskUpperBody.feature.selectDof(21,True)
robot.taskUpperBody.feature.selectDof(22,True)
robot.taskUpperBody.feature.selectDof(23,True)
robot.taskUpperBody.feature.selectDof(24,True)
robot.taskUpperBody.feature.selectDof(25,True)
robot.taskUpperBody.feature.selectDof(26,True)
robot.taskUpperBody.feature.selectDof(27,True)
robot.taskUpperBody.feature.selectDof(28,True)
robot.taskUpperBody.feature.selectDof(29,True)
robot.taskUpperBody.feature.selectDof(30,True)
robot.taskUpperBody.feature.selectDof(31,True)
robot.taskUpperBody.feature.selectDof(32,True)
robot.taskUpperBody.feature.selectDof(33,True)
robot.taskUpperBody.feature.selectDof(34,True)
robot.taskUpperBody.feature.selectDof(35,True)
robot.taskUpperBody.feature.selectDof(36,True)
robot.taskUpperBody.feature.selectDof(37,True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- CONTACTS
robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position)
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position)
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.wp.comDes,robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 10
robot.taskCom.feature.selec.value = '100'

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist',robot.dynamic,'WT',robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position)
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- Plug SOT control to device through control manager
plug(robot.sot.control,robot.cm.ctrl_sot_input)
plug(robot.cm.u_safe,robot.device.control)
# plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.taskRP.task.name)
robot.sot.push(robot.taskRR.task.name)
robot.sot.push(robot.taskLP.task.name)
robot.sot.push(robot.taskLR.task.name)
robot.sot.push(robot.keepWaist.task.name)

# --- Fix robot.dynamic inputs
plug(robot.device.velocity,robot.dynamic.velocity)
from dynamic_graph.sot.core import Derivator_of_Vector
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.device.velocity,robot.dvdt.sin)
plug(robot.dvdt.sout,robot.dynamic.acceleration)

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')

create_topic(robot.publisher, robot.wp, 'comDes', robot = robot, data_type='vector')                      # desired CoM

create_topic(robot.publisher, robot.cdc_estimator, 'c', robot = robot, data_type='vector')                # estimated CoM
create_topic(robot.publisher, robot.cdc_estimator, 'dc', robot = robot, data_type='vector')               # estimated CoM velocity

create_topic(robot.publisher, robot.dynamic, 'com', robot = robot, data_type='vector')                    # resulting SOT CoM

create_topic(robot.publisher, robot.dcm_control, 'dcmDes', robot = robot, data_type='vector')             # desired DCM
create_topic(robot.publisher, robot.estimator, 'dcm', robot = robot, data_type='vector')                  # estimated DCM

create_topic(robot.publisher, robot.dcm_control, 'zmpDes', robot = robot, data_type='vector')             # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', robot = robot, data_type='vector')                    # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot = robot, data_type='vector')              # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot = robot, data_type='vector')             # reference ZMP

create_topic(robot.publisher, robot.dcm_control, 'wrenchRef', robot = robot, data_type='vector')          # unoptimized reference wrench
create_topic(robot.publisher, robot.distribute, 'wrenchLeft', robot = robot, data_type='vector')          # reference left wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRight', robot = robot, data_type='vector')         # reference right wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRef', robot = robot, data_type='vector')           # optimized reference wrench

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot = robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot = robot, data_type='vector') # calibrated right wrench

create_topic(robot.publisher, robot.pitchController, 'tauL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDesL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'dqRefL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDesR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'dqRefR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauSum', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDiff', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDesSum', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDesDiff', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.rollController, 'tauL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'dqRefL', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'dqRefR', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauSum', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDiff', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesSum', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesDiff', robot = robot, data_type='vector')

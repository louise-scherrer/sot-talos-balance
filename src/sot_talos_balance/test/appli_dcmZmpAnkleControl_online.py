# flake8: noqa

# Try to implement the wrench distribution and ankle admittance controller of Stephane Caron into the dcmZmpControl_online simulation - June 2020

import sys

from math import sqrt

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, Derivator_of_Vector, FeaturePosture, MatrixHomoToPoseQuaternion, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio # replaces former 'dynamics_pinocchio'
from dynamic_graph.tracer_real_time import TracerRealTime
from rospkg import RosPack

from dynamic_graph.sot.core.operator import MatrixHomoToPoseRollPitchYaw


import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import sot_talos_balance.talos.control_manager_conf as cm_conf
import sot_talos_balance.talos.ft_calibration_conf as ft_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import *
from dynamic_graph.sot.pattern_generator import PatternGenerator

# added for wrench distribution & ankle admittance control
import sot_talos_balance.talos.distribute_conf as distribute_conf
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint


cm_conf.CTRL_MAX = 100.0  # temporary hack, used to be 10.0 in former versions but not enough anymore

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Pendulum parameters
robot_name = 'robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
#mass = 90.2722: when models inconsistent between WPG and simu (different urdf), this is the PG's robot weight

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

rospack = RosPack()

# -------------------------- PATTERN GENERATOR --------------------------

robot.pg = PatternGenerator('pg')

talos_data_folder = rospack.get_path('talos_data')
robot.pg.setURDFpath(talos_data_folder+'/urdf/talos_reduced_wpg.urdf')
robot.pg.setSRDFpath(talos_data_folder+'/srdf/talos_wpg.srdf')

robot.pg.buildModel()

robot.pg.parseCmd(":samplingperiod 0.005")
robot.pg.parseCmd(":previewcontroltime 1.6")
robot.pg.parseCmd(":omega 0.0")
robot.pg.parseCmd(':stepheight 0.05')
robot.pg.parseCmd(':doublesupporttime 0.2')
robot.pg.parseCmd(':singlesupporttime 1.0')
robot.pg.parseCmd(":armparameters 0.5")
robot.pg.parseCmd(":LimitsFeasibility 0.0")
robot.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
robot.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.7 3.0")
robot.pg.parseCmd(":UpperBodyMotionParameters -0.1 -1.0 0.0")
robot.pg.parseCmd(":comheight 0.876681")
robot.pg.parseCmd(":setVelReference  0.1 0.0 0.0")

robot.pg.parseCmd(":SetAlgoForZmpTrajectory Naveau")

plug(robot.dynamic.position,robot.pg.position)
plug(robot.dynamic.com, robot.pg.com)
plug(robot.dynamic.LF, robot.pg.leftfootcurrentpos)
plug(robot.dynamic.RF, robot.pg.rightfootcurrentpos)
robotDim = len(robot.dynamic.velocity.value)
robot.pg.motorcontrol.value = robotDim*(0,)
robot.pg.zmppreviouscontroller.value = (0,0,0)

robot.pg.initState()

robot.pg.parseCmd(':setDSFeetDistance 0.162')

robot.pg.parseCmd(':NaveauOnline')
robot.pg.parseCmd(':numberstepsbeforestop 2')
robot.pg.parseCmd(':setfeetconstraint XY 0.091 0.0489')

robot.pg.parseCmd(':deleteallobstacles')
robot.pg.parseCmd(':feedBackControl false')
robot.pg.parseCmd(':useDynamicFilter true')

robot.pg.velocitydes.value=(0.0,0.0,0.0) # Start stopped

# -------------------------- TRIGGER --------------------------

robot.triggerPG= BooleanIdentity('triggerPG')
robot.triggerPG.sin.value = 0
plug(robot.triggerPG.sout,robot.pg.trigger)

# -------------------------- Interface with controller entities --------------------------

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega

plug(robot.pg.waistattitudematrixabsolute, wp.waist) # New name for waistattitudematrix changed in pg.cpp in sot-pattern-generator
plug(robot.pg.leftfootref, wp.footLeft)
plug(robot.pg.rightfootref, wp.footRight)
plug(robot.pg.comref, wp.com)
plug(robot.pg.dcomref, wp.vcom)
plug(robot.pg.ddcomref, wp.acom)
plug(robot.pg.contactphase, wp.phase) # for ankle admittance controller
# rho set to constant for now in ankle controller, not used anymore with implementation of LeftFootRatio output signal in entity distribute-wrench
wp.rho.value = 0.5

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

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.base_estimator.q, robot.rdynamic.position)
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

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [8.0] * 3 # 8.0 de base
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

# === Test adding Wrench Distribution AND Ankle admittance controller following [Caron 2019] method
# --- Wrench distribution based on appli_dcmZmpControl_distribute.py
distribute = create_distribute_wrench(distribute_conf)
plug(robot.e2q.quaternion, distribute.q)
plug(robot.dcm_control.wrenchRef, distribute.wrenchDes)
plug(robot.wp.rhoDes, distribute.rho)
plug(robot.wp.phaseDes, distribute.phase)

plug(robot.wp.comDes, distribute.comDes)

# Adding Foot signals and ZMP Des for left foot ratio computation (new rho formulation)
plug(robot.wp.footLeftDes, distribute.footLeftDes)
plug(robot.wp.footRightDes, distribute.footRightDes)
distribute.zmpDes.value = robot.wp.zmpDes.value # should later be plugged to dcm_controller

distribute.init(robot_name)
robot.wrenchDistributor = distribute # required by create_ankle_admittance_controller


# --- CoM admittance controller
Kp_adm = [0.0, 0.0, 0.0]  # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp, com_admittance_control.zmp)
#com_admittance_control.zmpDes.value = robot.wp.zmpDes.value  # should be plugged to robot.dcm_control.zmpRef

plug(robot.wrenchDistributor.zmpRef, com_admittance_control.zmpDes) # direct plug

plug(robot.wp.acomDes, com_admittance_control.ddcomDes)

com_admittance_control.init(dt)
com_admittance_control.setState(robot.wp.comDes.value, [0.0, 0.0, 0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [15.0, 15.0, 0.0]  # this value is employed later

# --- Ankle admittance
GainsXY = [0.0,0.0]

# Left foot
robot.leftAnkleController = create_ankle_admittance_controller(GainsXY, robot, "left", "leftController", dt) # 0 de base dans les simus, test valeurs Caron = 0.1

# Right foot
robot.rightAnkleController = create_ankle_admittance_controller(GainsXY, robot, "right", "rightController", dt)


GainsXY = [0.01,0.01] # value passed later in test -0.1 if controller opposite to usual diff value
# largest stable value 0.001 on SSP, 0.007 DSP as of July 2020, model issue not solved

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all', 'sot_input')
robot.cm.addEmergencyStopSIN('zmp')

# -------------------------- SOT CONTROL --------------------------

# --- Upper body
robot.taskUpperBody = Task('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

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
plug(robot.leftAnkleController.poseDes, robot.contactLF.featureDes.position)

locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)

plug(robot.rightAnkleController.poseDes, robot.contactRF.featureDes.position)
# test 02.07, il faudrait dPoseLF désiré par PG pour additionner à vDes (nulle dès que contact, mais entre ?)
#plug(robot.rightAnkleController.vDes, robot.contactRF.featureDes.velocity)

locals()['contactRF'] = robot.contactRF

# --- COM height
robot.taskComH = MetaTaskKineCom(robot.dynamic, name='comH')
plug(robot.wp.comDes, robot.taskComH.featureDes.errorIN)
robot.taskComH.task.controlGain.value = 100.
robot.taskComH.feature.selec.value = '100'

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.com_admittance_control.comRef, robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef, robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 0 #should maybe be 100 see dcm_zmp_control_flex
robot.taskCom.task.setWithDerivative(True)
robot.taskCom.feature.selec.value = '011'

# --- Waist

robot.keepWaist = MetaTaskKine6d('keepWaist', robot.dynamic, 'WT', robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position) # 'waistDes' is output signal of dummy with entry signal 'waist'
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
robot.sot.push(robot.taskComH.task.name)
robot.sot.push(robot.taskCom.task.name)
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

create_topic(robot.publisher, robot.pg, 'comref', robot = robot, data_type='vector')                      # desired CoM
create_topic(robot.publisher, robot.pg, 'dcomref', robot = robot, data_type='vector')
#create_topic(robot.publisher, robot.wp, 'waistDes', robot = robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.wp, 'waist', robot = robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.keepWaist.featureDes, 'position', robot = robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.dynamic, 'WT', robot = robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.pg, 'waistattitudematrixabsolute', robot = robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.pg, 'leftfootref', robot = robot, data_type ='matrixHomo')
create_topic(robot.publisher, robot.wp, 'footLeft', robot = robot, data_type ='matrixHomo')
create_topic(robot.publisher, robot.wp, 'footLeftDes', robot = robot, data_type ='matrixHomo')
create_topic(robot.publisher, robot.pg, 'rightfootref', robot = robot, data_type ='matrixHomo')
create_topic(robot.publisher, robot.wp, 'footRightDes', robot = robot, data_type ='matrixHomo')
create_topic(robot.publisher, robot.wp, 'footLeft', robot = robot, data_type ='matrixHomo')

create_topic(robot.publisher, robot.pg, 'zmpref', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.wp, 'zmpDes', robot = robot, data_type ='vector')
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot = robot, data_type ='vector')
create_topic(robot.publisher, robot.dcm_control, 'zmpDes', robot = robot, data_type ='vector')

create_topic(robot.publisher, robot.pg, 'contactphase', robot=robot, data_type='int')


create_topic(robot.publisher, robot.pg, 'extForces', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.device, 'state', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.stf, 'q', robot = robot, data_type='vector')

# distribute stuff
create_topic(robot.publisher, robot.dcm_control, 'wrenchRef', robot=robot, data_type='vector')  # unoptimized reference wrench
create_topic(robot.publisher, robot.wrenchDistributor, 'wrenchLeft', robot=robot, data_type='vector')  # reference left wrench
create_topic(robot.publisher, robot.wrenchDistributor, 'wrenchRight', robot=robot, data_type='vector')  # reference right wrench
create_topic(robot.publisher, robot.wrenchDistributor, 'wrenchRef', robot=robot, data_type='vector')  # optimized reference wrench
create_topic(robot.publisher, robot.wrenchDistributor, 'zmpRef', robot=robot, data_type='vector') # plugged to com_adm_ctrler
create_topic(robot.publisher, robot.wrenchDistributor, 'leftFootRatio', robot=robot, data_type='double')
create_topic(robot.publisher, robot.wrenchDistributor, 'copRight', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'copLeft', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'surfaceWrenchLeft', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'surfaceWrenchRight', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.wp, 'comDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'staticFeetForces', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.zmp_estimator, 'poseLeft', robot=robot, data_type='matrixHomo') # produced by real_dynamics
create_topic(robot.publisher, robot.zmp_estimator, 'poseRight', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.zmp_estimator, 'copLeft', robot=robot, data_type='matrixHomo') # produced by real_dynamics
create_topic(robot.publisher, robot.zmp_estimator, 'copRight', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.wp, 'phaseDes', robot=robot, data_type='int')

create_topic(robot.publisher, robot.leftAnkleController, 'dRP', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightAnkleController, 'dRP', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.leftAnkleController, 'vDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rightAnkleController, 'vDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.leftAnkleController, 'poseDes', robot=robot, data_type='matrixHomo') # remettre matrix homo
create_topic(robot.publisher, robot.rightAnkleController, 'poseDes', robot=robot, data_type='matrixHomo') # remettre matrix homo
create_topic(robot.publisher, robot.LeftFootDesAnkleCtrler, 'sout', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')               # measured right wrench

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot=robot, data_type='vector')  # calibrated right wrench

create_topic(robot.publisher, robot.dynamic, 'LF', robot=robot, data_type='matrixHomo')  # left foot
create_topic(robot.publisher, robot.dynamic, 'RF', robot=robot, data_type='matrixHomo')  # right foot
#create_topic(robot.publisher, robot.contactLF, 'featureDes', robot=robot, data_type='vector') # marche pas
create_topic(robot.publisher, robot.contactLF.featureDes, 'position', robot=robot, data_type='matrixHomo')
#create_topic(robot.publisher, robot.contactLF.featureDes, 'velocity', robot=robot, data_type='vector')
#create_topic(robot.publisher, robot.contactLFRef, 'sout', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.contactLF.feature, 'position', robot=robot, data_type='matrixHomo')
#create_topic(robot.publisher, robot.contactLF.feature, 'velocity', robot=robot, data_type='vector') # -> not plugged
create_topic(robot.publisher, robot.sot, 'control', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.pg, 'velocitydes', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot=robot, data_type='vector') # new reference

create_topic(robot.publisher, robot.estimator, 'dcm', robot=robot, data_type='vector') #estimated/measured DCM
create_topic(robot.publisher, robot.wp, 'dcmDes', robot=robot, data_type='vector')# reference DCM from PG

## --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80 * (2**20))

robot.tracer.open('/tmp', 'dg_', '.dat')

robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.pg, 'waistattitudeabsolute')


addTrace(robot.tracer, robot.wp, 'comDes')  # desired CoM

addTrace(robot.tracer, robot.cdc_estimator, 'c')  # estimated CoM
addTrace(robot.tracer, robot.cdc_estimator, 'dc')  # estimated CoM velocity

addTrace(robot.tracer, robot.pg, 'comref')
addTrace(robot.tracer, robot.pg, 'dcomref')
addTrace(robot.tracer, robot.pg, 'ddcomref')

addTrace(robot.tracer, robot.pg, 'rightfootref')
addTrace(robot.tracer, robot.pg, 'leftfootref')

addTrace(robot.tracer, robot.pg, 'rightfootcontact')
addTrace(robot.tracer, robot.pg, 'leftfootcontact')
addTrace(robot.tracer, robot.pg, 'SupportFoot')

addTrace(robot.tracer, robot.dynamic, 'com')  # resulting SOT CoM

addTrace(robot.tracer, robot.dynamic, 'LF')  # left foot
addTrace(robot.tracer, robot.dynamic, 'RF')  # right foot

create_topic(robot.publisher, robot.wp, 'phaseDes', robot=robot, data_type='int')

create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'surfaceWrenchLeft', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wrenchDistributor, 'surfaceWrenchRight', robot=robot, data_type='vector')

robot.tracer.start()

from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.talos.parameter_server_conf   as param_server_conf
import sot_talos_balance.talos.control_manager_conf    as cm_conf
import sot_talos_balance.talos.base_estimator_conf     as base_estimator_conf
import sot_talos_balance.talos.ft_calibration_conf     as ft_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from math import sqrt
import numpy as np

from dynamic_graph.tracer_real_time import TracerRealTime

from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

from dynamic_graph.sot.pattern_generator import PatternGenerator

cm_conf.CTRL_MAX = 10.0 # temporary hack

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

# -------------------------- PATTERN GENERATOR --------------------------

robot.pg = PatternGenerator('pg')
robot.pg.setURDFpath("/local/imaroger/sot-pattern-generator/talos_reduced_wpg.urdf")
robot.pg.setSRDFpath("/local/imaroger/sot-pattern-generator/talos_wpg.srdf")

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
#plug(robot.dynamic.com, robot.pg.comStateSIN)
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

robot.pg.velocitydes.value=(0.1,0.0,0.0)

#robot.pg.displaySignals()

# -------------------------- TRIGGER -------------------------- 

robot.triggerPG= BooleanIdentity('triggerPG')
robot.triggerPG.sin.value = 0
plug(robot.triggerPG.sout,robot.pg.trigger)

# -------------------------- Interface with controller entities --------------------------

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
# #wp.displaySignals()
wp.omega.value = omega

plug(robot.pg.waistattitudematrix, wp.waist)
plug(robot.pg.leftfootref, wp.footLeft)
plug(robot.pg.rightfootref, wp.footRight)
plug(robot.pg.comref, wp.com)
plug(robot.pg.dcomref, wp.vcom)
plug(robot.pg.ddcomref, wp.acom)
#plug(robot.zmpTrajGen.x, wp.zmp)

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

from dynamic_graph.sot.core import MatrixHomoToPoseQuaternion
robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.LF, robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.RF, robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# robot.be_filters              = create_be_filters(robot, dt)

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q,e2q.euler)

robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.base_estimator.q,robot.rdynamic.position)

robot.rdynamic.velocity.value = [0.0]*robotDim
robot.rdynamic.acceleration.value = [0.0]*robotDim

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.base_estimator.v, cdc_estimator.v)

robot.cdc_estimator = cdc_estimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
plug(robot.wp.omegaDes, estimator.omega)

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
Kp_dcm = [8.0]*3
Ki_dcm = [0.0,0.0,0.0] # zero (to be set later)
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
plug(robot.wp.omegaDes, dcm_controller.omega)

plug(robot.cdc_estimator.c,dcm_controller.com)
plug(robot.estimator.dcm,dcm_controller.dcm)

plug(robot.wp.zmpDes, dcm_controller.zmpDes)
plug(robot.wp.dcmDes, dcm_controller.dcmDes)

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [1.0,1.0,1.0] # this value is employed later

# --- CoM admittance controller
Kp_adm = [0.0,0.0,0.0] # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp,com_admittance_control.zmp)
com_admittance_control.zmpDes.value = robot.wp.zmpDes.value     # should be plugged to robot.dcm_control.zmpRef
plug(robot.wp.acomDes, com_admittance_control.ddcomDes)

com_admittance_control.init(dt)
com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [15.0,15.0,0.0] # this value is employed later

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all','sot_input')
robot.cm.addEmergencyStopSIN('zmp')

# -------------------------- SOT CONTROL --------------------------

# --- Upper body
robot.taskUpperBody = Task ('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

robotDim = robot.dynamic.getDimension() # 38
for i in range(18, robotDim):
    robot.taskUpperBody.feature.selectDof(i,True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position) #.errorIN?
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position) #.errorIN?
locals()['contactRF'] = robot.contactRF

# --- COM height
robot.taskComH = MetaTaskKineCom(robot.dynamic, name='comH')
plug(robot.wp.comDes,robot.taskComH.featureDes.errorIN)
robot.taskComH.task.controlGain.value = 100.
robot.taskComH.feature.selec.value = '100'

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.com_admittance_control.comRef,robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef,robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 100.
robot.taskCom.task.setWithDerivative(True)
robot.taskCom.feature.selec.value = '011'

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

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskComH.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)
# robot.sot.push(robot.taskPos.name)
# robot.device.control.recompute(0) # this crashes as it employs joint sensors which are not ready yet

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

create_topic(robot.publisher, robot.pg, 'comref', robot = robot, data_type='vector')                      # desired CoM
create_topic(robot.publisher, robot.pg, 'dcomref', robot = robot, data_type='vector')


# --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/local/imaroger/sot-talos-balance/python/sot_talos_balance/test_result','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.wp, 'comDes')                      
addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.pg, 'comref')
addTrace(robot.tracer, robot.pg, 'dcomref')    
addTrace(robot.tracer, robot.pg, 'ddcomref')                   
addTrace(robot.tracer, robot.cdc_estimator, 'c')             
addTrace(robot.tracer, robot.cdc_estimator, 'dc')              

addTrace(robot.tracer, robot.pg, 'rightfootref')
addTrace(robot.tracer, robot.pg, 'leftfootref')
addTrace(robot.tracer, robot.dynamic, 'LF')
addTrace(robot.tracer, robot.dynamic, 'RF')

addTrace(robot.tracer, robot.pg, 'rightfootcontact')
addTrace(robot.tracer, robot.pg, 'leftfootcontact')
addTrace(robot.tracer, robot.pg, 'SupportFoot')
robot.tracer.start()


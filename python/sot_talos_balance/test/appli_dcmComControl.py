# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.tracer_real_time import TracerRealTime

from sot_talos_balance.create_entities_utils import *

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Dummy estimator
robot.estimator = create_dummy_dcm_estimator(robot)

# --- DCM controller
Kp_dcm = [500.0, 500.0, 500.0]
Ki_dcm = [1.0, 1.0, 0.0]  # to be set later
robot.dcm_control = create_dcm_com_controller(Kp_dcm, [0.0] * 3, dt, robot, robot.estimator.dcm)

# --- Admittance controller
# --- Gains = 0, ddcomDes = ddcomRef --> admittance controller is a double integrator
robot.com_admittance_control = create_com_admittance_controller([0.0] * 3, dt, robot)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
plug(robot.com_admittance_control.comRef, robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef, robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 0
robot.taskCom.task.setWithDerivative(True)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.device.control.recompute(0)

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
robot.device.after.addSignal('{0}.zmpRef'.format(robot.dcm_control.name))
robot.device.after.addSignal('{0}.zmp'.format(robot.dynamic.name))  # why needed?
robot.device.after.addSignal('{0}.comRef'.format(robot.com_admittance_control.name))  # why needed?

addTrace(robot.tracer, robot.dynamic, 'zmp')
addTrace(robot.tracer, robot.dcm_control, 'zmpRef')
addTrace(robot.tracer, robot.estimator, 'dcm')
addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.com_admittance_control, 'comRef')

# SIMULATION

robot.tracer.start()

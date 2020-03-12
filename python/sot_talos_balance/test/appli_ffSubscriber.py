# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.ros import RosSubscribe
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.tracer_real_time import TracerRealTime

from sot_talos_balance.create_entities_utils import addTrace, create_com_trajectory_generator, dump_tracer

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep
robot.comTrajGen = create_com_trajectory_generator(dt, robot)

robot.subscriber = RosSubscribe("base_subscriber")
robot.subscriber.add("vector", "position", "/sot/base_link/position")
robot.subscriber.add("vector", "velocity", "/sot/base_link/velocity")

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10

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

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)
plug(robot.comTrajGen.x, robot.taskCom.featureDes.errorIN)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.device.control.recompute(0)

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
robot.device.after.addSignal('{0}.position'.format(robot.subscriber.name))  # force recalculation
robot.device.after.addSignal('{0}.velocity'.format(robot.subscriber.name))  # force recalculation

addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.subscriber, 'position')
addTrace(robot.tracer, robot.subscriber, 'velocity')

robot.tracer.start()

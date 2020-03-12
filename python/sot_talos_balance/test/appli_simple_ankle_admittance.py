# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, FeaturePosture, Task
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.operator import Selec_of_vector

from sot_talos_balance.meta_task_joint import MetaTaskKineJoint
from sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController

N_JOINTS = 32
N_CONFIG = N_JOINTS + 6

robot.timeStep = robot.device.getTimeStep()
timeStep = robot.timeStep

RightPitchJoint = 10
RightRollJoint = 11
LeftPitchJoint = 4
LeftRollJoint = 5

Kp = [0.00005]

# --- Ankle admittance foot
# --- RIGHT ANKLE PITCH
controller = SimpleAdmittanceController("rightPitchAnkleController")
controller.Kp.value = Kp

robot.stateselecRP = Selec_of_vector("stateselecRP")
robot.stateselecRP.selec(RightPitchJoint + 6, RightPitchJoint + 7)
plug(robot.device.state, robot.stateselecRP.sin)
plug(robot.stateselecRP.sout, controller.state)

robot.tauselecRP = Selec_of_vector("tauselecRP")
robot.tauselecRP.selec(RightPitchJoint, RightPitchJoint + 1)
plug(robot.device.ptorque, robot.tauselecRP.sin)
plug(robot.tauselecRP.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[RightPitchJoint + 6]])
robot.rightPitchAnkleController = controller

robot.rightAnklePitchTask = MetaTaskKineJoint(robot.dynamic, RightPitchJoint)
robot.rightAnklePitchTask.task.controlGain.value = 0
robot.rightAnklePitchTask.task.setWithDerivative(True)
plug(robot.rightPitchAnkleController.qRef, robot.rightAnklePitchTask.featureDes.errorIN)
plug(robot.rightPitchAnkleController.dqRef, robot.rightAnklePitchTask.featureDes.errordotIN)

# --- RIGHT ANKLE ROLL
controller = SimpleAdmittanceController("rightRollAnkleController")
controller.Kp.value = Kp

robot.stateselecRR = Selec_of_vector("stateselecRR")
robot.stateselecRR.selec(RightRollJoint + 6, RightRollJoint + 7)
plug(robot.device.state, robot.stateselecRR.sin)
plug(robot.stateselecRR.sout, controller.state)

robot.tauselecRR = Selec_of_vector("tauselecRR")
robot.tauselecRR.selec(RightRollJoint, RightRollJoint + 1)
plug(robot.device.ptorque, robot.tauselecRR.sin)
plug(robot.tauselecRR.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[RightRollJoint + 6]])
robot.rightRollAnkleController = controller

robot.rightAnkleRollTask = MetaTaskKineJoint(robot.dynamic, RightRollJoint)
robot.rightAnkleRollTask.task.controlGain.value = 0
robot.rightAnkleRollTask.task.setWithDerivative(True)
plug(robot.rightRollAnkleController.qRef, robot.rightAnkleRollTask.featureDes.errorIN)
plug(robot.rightRollAnkleController.dqRef, robot.rightAnkleRollTask.featureDes.errordotIN)

# --- LEFT ANKLE PITCH
controller = SimpleAdmittanceController("leftPitchAnkleController")
controller.Kp.value = Kp

robot.stateselecLP = Selec_of_vector("stateselecLP")
robot.stateselecLP.selec(LeftPitchJoint + 6, LeftPitchJoint + 7)
plug(robot.device.state, robot.stateselecLP.sin)
plug(robot.stateselecLP.sout, controller.state)

robot.tauselecLP = Selec_of_vector("tauselecLP")
robot.tauselecLP.selec(LeftPitchJoint, LeftPitchJoint + 1)
plug(robot.device.ptorque, robot.tauselecLP.sin)
plug(robot.tauselecLP.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[LeftPitchJoint + 6]])
robot.leftPitchAnkleController = controller

robot.leftAnklePitchTask = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint)
robot.leftAnklePitchTask.task.controlGain.value = 0
robot.leftAnklePitchTask.task.setWithDerivative(True)
plug(robot.leftPitchAnkleController.qRef, robot.leftAnklePitchTask.featureDes.errorIN)
plug(robot.leftPitchAnkleController.dqRef, robot.leftAnklePitchTask.featureDes.errordotIN)

# --- LEFT ANKLE ROLL
controller = SimpleAdmittanceController("leftRollAnkleController")
controller.Kp.value = Kp

robot.stateselecLR = Selec_of_vector("stateselecLP")
robot.stateselecLR.selec(LeftRollJoint + 6, LeftRollJoint + 7)
plug(robot.device.state, robot.stateselecLR.sin)
plug(robot.stateselecLR.sout, controller.state)

robot.tauselecLR = Selec_of_vector("tauselecLP")
robot.tauselecLR.selec(LeftRollJoint, LeftRollJoint + 1)
plug(robot.device.ptorque, robot.tauselecLR.sin)
plug(robot.tauselecLR.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[LeftRollJoint + 6]])
robot.leftRollAnkleController = controller

robot.leftAnkleRollTask = MetaTaskKineJoint(robot.dynamic, LeftRollJoint)
robot.leftAnkleRollTask.task.controlGain.value = 0
robot.leftAnkleRollTask.task.setWithDerivative(True)
plug(robot.leftRollAnkleController.qRef, robot.leftAnkleRollTask.featureDes.errorIN)
plug(robot.leftRollAnkleController.dqRef, robot.leftAnkleRollTask.featureDes.errordotIN)

# -------------------------- SOT CONTROL --------------------------
# --- Posture
robot.taskPosture = Task('taskPosture')
robot.taskPosture.feature = FeaturePosture('featurePosture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

for i in range(18, 38):
    robot.taskPosture.feature.selectDof(i, True)

robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

# --- CONTACTS
# define contactLF and contactRF
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

# --- SOT

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.taskPosture.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.device.control.recompute(0)

# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish, RosSubscribe
from dynamic_graph.sot.core import SOT, FeaturePosture, Task
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.operator import Add_of_vector, Selec_of_vector

from sot_talos_balance.coupled_admittance_controller import CoupledAdmittanceController
from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint

N_JOINTS = 32
N_CONFIG = N_JOINTS + 6

robot.timeStep = robot.device.getTimeStep()
timeStep = robot.timeStep

RightPitchJoint = 10
RightRollJoint = 11
LeftPitchJoint = 4
LeftRollJoint = 5

kSumPitch = [-0.000]
kSumRoll = [0.001]
kDiffPitch = [-0.000]
kDiffRoll = [0.001]
tauDesRoll = [0.0]
tauDesPitch = [0.0]

# ----------  ADMITTANCE CONTROLLERS ----------
# --- PITCH
robot.tauselecRP = Selec_of_vector("tauselecRP")
robot.tauselecRP.selec(RightPitchJoint, RightPitchJoint + 1)
plug(robot.device.ptorque, robot.tauselecRP.sin)

robot.tauselecLP = Selec_of_vector("tauselecLP")
robot.tauselecLP.selec(LeftPitchJoint, LeftPitchJoint + 1)
plug(robot.device.ptorque, robot.tauselecLP.sin)

robot.pitchController = CoupledAdmittanceController("pitchController")
robot.pitchController.kSum.value = kSumPitch
robot.pitchController.kDiff.value = kDiffPitch
robot.pitchController.tauDesL.value = tauDesPitch
robot.pitchController.tauDesR.value = tauDesPitch
plug(robot.tauselecRP.sout, robot.pitchController.tauR)
plug(robot.tauselecLP.sout, robot.pitchController.tauL)

# --- ROLL
robot.tauselecRR = Selec_of_vector("tauselecRR")
robot.tauselecRR.selec(RightRollJoint, RightRollJoint + 1)
plug(robot.device.ptorque, robot.tauselecRR.sin)

robot.tauselecLR = Selec_of_vector("tauselecLR")
robot.tauselecLR.selec(LeftRollJoint, LeftRollJoint + 1)
plug(robot.device.ptorque, robot.tauselecLR.sin)

robot.rollController = CoupledAdmittanceController("rollController")
robot.rollController.kSum.value = kSumRoll
robot.rollController.kDiff.value = kDiffRoll
robot.rollController.tauDesL.value = tauDesRoll
robot.rollController.tauDesR.value = tauDesRoll
plug(robot.tauselecRR.sout, robot.rollController.tauR)
plug(robot.tauselecLR.sout, robot.rollController.tauL)

# ----------  TASKS ----------
# RIGHT PITCH
robot.taskRP = MetaTaskKineJoint(robot.dynamic, RightPitchJoint + 6)
robot.taskRP.task.controlGain.value = 0
robot.taskRP.task.setWithDerivative(True)
robot.taskRP.featureDes.errorIN.value = [0.0]
plug(robot.pitchController.dqRefR, robot.taskRP.featureDes.errordotIN)

#  RIGHT ROLL
robot.taskRR = MetaTaskKineJoint(robot.dynamic, RightRollJoint + 6)
robot.taskRR.task.controlGain.value = 0
robot.taskRR.task.setWithDerivative(True)
robot.taskRR.featureDes.errorIN.value = [0.0]
plug(robot.rollController.dqRefR, robot.taskRR.featureDes.errordotIN)

# --- LEFT PITCH
robot.taskLP = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint + 6)
robot.taskLP.task.controlGain.value = 0
robot.taskLP.task.setWithDerivative(True)
robot.taskLP.featureDes.errorIN.value = [0.0]
plug(robot.pitchController.dqRefL, robot.taskLP.featureDes.errordotIN)

# --- LEFT ROLL
robot.taskLR = MetaTaskKineJoint(robot.dynamic, LeftRollJoint + 6)
robot.taskLR.task.controlGain.value = 0
robot.taskLR.task.setWithDerivative(True)
robot.taskLR.featureDes.errorIN.value = [0.0]
plug(robot.rollController.dqRefL, robot.taskLR.featureDes.errordotIN)

# -------------------------- SOT CONTROL --------------------------
# --- Posture
robot.taskPosture = Task('taskPosture')
robot.taskPosture.feature = FeaturePosture('featurePosture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

for i in range(6, 38):
    robot.taskPosture.feature.selectDof(i, True)

robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

# --- SOT
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.taskRP.task.name)
robot.sot.push(robot.taskLP.task.name)
robot.sot.push(robot.taskRR.task.name)
robot.sot.push(robot.taskLR.task.name)
robot.sot.push(robot.taskPosture.name)
robot.device.control.recompute(0)

# -------------------------- PLOTS --------------------------
# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')

create_topic(robot.publisher, robot.pitchController, 'tauL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauR', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauR', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.pitchController, 'tauDesL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'tauDesR', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'tauDesR', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.pitchController, 'dqRefL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'dqRefL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pitchController, 'dqRefR', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rollController, 'dqRefR', robot=robot, data_type='vector')

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("ankle_joint_subscriber")

robot.subscriber.add("vector", "tauRP", "/sot/pitchController/tauL")
robot.subscriber.add("vector", "tauRR", "/sot/rollController/tauR")
robot.subscriber.add("vector", "tauLP", "/sot/pitchController/tauL")
robot.subscriber.add("vector", "tauLR", "/sot/rollController/tauR")

robot.subscriber.add("vector", "tauDesRP", "/sot/pitchController/tauDesL")
robot.subscriber.add("vector", "tauDesRR", "/sot/rollController/tauDesR")
robot.subscriber.add("vector", "tauDesLP", "/sot/pitchController/tauDesL")
robot.subscriber.add("vector", "tauDesLR", "/sot/rollController/tauDesR")

robot.subscriber.add("vector", "dqRefRP", "/sot/pitchController/dqRefL")
robot.subscriber.add("vector", "dqRefRR", "/sot/rollController/dqRefR")
robot.subscriber.add("vector", "dqRefLP", "/sot/pitchController/dqRefL")
robot.subscriber.add("vector", "dqRefLR", "/sot/rollController/dqRefR")

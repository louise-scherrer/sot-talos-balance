# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd

import sot_talos_balance.talos.control_manager_conf as cm_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import *

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep
robot.comTrajGen = create_com_trajectory_generator(dt, robot)

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(0)
robot.contactLF.keep()
robot.contactLF.task.setWithDerivative(True)
robot.contactLF.featureDes.velocity.value = [0.] * 6
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(0)
robot.contactRF.keep()
robot.contactRF.task.setWithDerivative(True)
robot.contactRF.featureDes.velocity.value = [0.] * 6
locals()['contactRF'] = robot.contactRF

robot.taskRH = MetaTaskKine6d('taskRH', robot.dynamic, 'RH', 'arm_right_7_joint')
robot.taskRH.feature.frame('desired')
robot.taskRH.gain.setConstant(0)
robot.taskRH.keep()
robot.taskRH.task.setWithDerivative(True)
robot.taskRH.featureDes.velocity.value = [0.] * 6
locals()['taskRH'] = robot.taskRH

robot.taskLH = MetaTaskKine6d('taskLH', robot.dynamic, 'LH', 'arm_left_7_joint')
robot.taskLH.feature.frame('desired')
robot.taskLH.gain.setConstant(0)
robot.taskLH.keep()
robot.taskLH.task.setWithDerivative(True)
robot.taskLH.featureDes.velocity.value = [0.] * 6
locals()['taskLH'] = robot.taskLH

#robot.taskRH = MetaTaskKine6d('taskRH', robot.dynamic, 'RH', 'arm_right_7_joint')
#handMgrip = np.eye(4)
#handMgrip[0:3, 3] = (0.1, 0, 0)
#robot.taskRH.opmodif = matrixToTuple(handMgrip)
#robot.taskRH.feature.frame('desired')
#robot.taskRH.feature.selec.value = '111111'
#robot.taskRH.task.setWithDerivative(True)
#robot.taskRH.task.controlGain.value = 0
#robot.taskRH.feature.position.value = np.eye(4)
#robot.taskRH.feature.velocity.value = [0., 0., 0., 0., 0., 0.]
#robot.taskRH.featureDes.position.value = np.eye(4)
#robot.taskRH.featureDes.velocity.value = [0., 0., 0., 0., 0., 0.]

#robot.taskLH = MetaTaskKine6d('taskLH', robot.dynamic, 'LH', 'arm_left_7_joint')
#handMgrip = np.eye(4)
#handMgrip[0:3, 3] = (0.1, 0, 0)
#robot.taskLH.opmodif = matrixToTuple(handMgrip)
#robot.taskLH.feature.frame('desired')
#robot.taskLH.feature.selec.value = '111111'
#robot.taskLH.task.setWithDerivative(True)
#robot.taskLH.task.controlGain.value = 0
#robot.taskLH.feature.position.value = np.eye(4)
#robot.taskLH.feature.velocity.value = [0., 0., 0., 0., 0., 0.]
#robot.taskLH.featureDes.position.value = np.eye(4)
#robot.taskLH.featureDes.velocity.value = [0., 0., 0., 0., 0., 0.]

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- Plug SOT control to device through control manager
plug(robot.sot.control, robot.device.control)

plug(robot.comTrajGen.x, robot.taskCom.featureDes.errorIN)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskRH.task.name)
robot.sot.push(robot.taskLH.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.device.control.recompute(0)

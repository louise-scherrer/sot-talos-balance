# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd

from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint

N_JOINTS = 32
N_CONFIG = N_JOINTS + 6

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

JOINT = 25
QJOINT = JOINT + 6

target = -10.0

# --- Joint
robot.taskJoint = MetaTaskKineJoint(robot.dynamic, QJOINT)
robot.taskJoint.featureDes.errorIN.value = [0.0]
robot.taskJoint.task.controlGain.value = 100

# --- filter
robot.device_filters = create_device_filters(robot, dt)

# --- Admittance controller
Kp = [0.5]
robot.admittance_control = create_joint_admittance_controller(JOINT, Kp, dt, robot)
plug(robot.admittance_control.qRef, robot.taskJoint.featureDes.errorIN)

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

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.device.control.recompute(0)

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.device_filters.torque_filter, 'x_filtered', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'ptorque', robot=robot, data_type='vector')

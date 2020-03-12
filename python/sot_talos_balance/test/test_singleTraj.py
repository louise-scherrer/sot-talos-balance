import os
from time import sleep

from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d

from sot_talos_balance.create_entities_utils import create_config_trajectory_generator
from sot_talos_balance.meta_task_config import MetaTaskKineConfig

N_JOINTS = 32
N_CONFIG = N_JOINTS + 6


def main(robot):
    robot.timeStep = robot.device.getTimeStep()
    dt = robot.timeStep
    robot.traj_gen = create_config_trajectory_generator(dt, robot)

    JOINT = 31
    QJOINT = JOINT + 6

    # --- Joint
    robot.taskJoint = MetaTaskKineConfig(robot.dynamic, [QJOINT])
    # robot.dynamic.com.recompute(0)
    robot.taskJoint.featureDes.errorIN.value = N_CONFIG * [0.0]
    robot.taskJoint.task.controlGain.value = 100

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

    robot.sot = SOT('sot')
    robot.sot.setSize(robot.dynamic.getDimension())
    plug(robot.sot.control, robot.device.control)

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.sot.push(robot.taskJoint.task.name)
    robot.device.control.recompute(0)

    plug(robot.traj_gen.x, robot.taskJoint.featureDes.errorIN)
    sleep(1.0)
    os.system('rosservice call /start_dynamic_graph')
    sleep(1.0)
    robot.traj_gen.move(QJOINT, -1.0, 1.0)
    sleep(1.1)
    robot.traj_gen.startSinusoid(QJOINT, 1.0, 8.0)
    sleep(10.0)
    robot.traj_gen.stop(QJOINT)

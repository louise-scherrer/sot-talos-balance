#!/usr/bin/python
# -*- coding: utf-8 -*-1
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom

from sot_talos_balance.create_entities_utils import (create_com_trajectory_generator, create_example,
                                                     create_parameter_server)
from sot_talos_balance.utils.sot_utils import Bunch


def get_default_conf():
    import sot_talos_balance.talos.control_manager_conf as param_server_conf
    conf = Bunch()
    conf.param_server = param_server_conf
    return conf


def main(robot, conf=None):
    if (conf is None):
        conf = get_default_conf()
    robot.timeStep = robot.device.getTimeStep()
    dt = robot.timeStep
    NJ = robot.dimension - 7
    robot.comTrajGen = create_com_trajectory_generator(dt, robot)

    # --- COM
    robot.taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 10

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

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.device.control.recompute(0)

    # --- ENTITIES
    robot.param_server = create_parameter_server(conf.param_server, dt)
    robot.example = create_example()

    # --- RUN SIMULATION
    # plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);
    # sleep(1.0);
    # os.system("rosservice call \start_dynamic_graph")
    # sleep(2.0);
    # robot.comTrajGen.move(1,-0.025,4.0);
    # sleep(5.0);
    # robot.comTrajGen.startSinusoid(1,0.05,8.0);
    # sleep(1.0)
    # robot.example.nbJoints.recompute(1)
    # # outputs the number of joints of the robot thanks to a pointer to the RobotUtil created in parameter server
    # print(robot.example.nbJoints.value)

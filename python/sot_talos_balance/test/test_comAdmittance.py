import os
from time import sleep

import matplotlib.pyplot as plt
import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.tracer_real_time import TracerRealTime

from sot_talos_balance.create_entities_utils import (addTrace, create_com_admittance_controller,
                                                     create_dummy_dcm_estimator, dump_tracer)


def main(robot):
    robot.timeStep = robot.device.getTimeStep()
    dt = robot.timeStep

    # --- COM
    robot.taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 10

    robot.estimator = create_dummy_dcm_estimator(robot)

    # --- Admittance controller
    Kp = [0.0, 0.0, 0.0]
    robot.com_admittance_control = create_com_admittance_controller(Kp, dt, robot)

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
    robot.sot.push(robot.taskCom.task.name)
    robot.device.control.recompute(0)

    # --- TRACER
    robot.tracer = TracerRealTime("zmp_tracer")
    robot.tracer.setBufferSize(80 * (2**20))
    robot.tracer.open('/tmp', 'dg_', '.dat')
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

    addTrace(robot.tracer, robot.dynamic, 'zmp')
    addTrace(robot.tracer, robot.estimator, 'dcm')

    # SIMULATION

    plug(robot.com_admittance_control.comRef, robot.taskCom.featureDes.errorIN)
    sleep(1.0)
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0)

    plug(robot.estimator.dcm, robot.com_admittance_control.zmpDes)

    robot.com_admittance_control.setState(robot.dynamic.com.value, [0.0, 0.0, 0.0])
    robot.com_admittance_control.Kp.value = [10.0, 10.0, 0.0]

    robot.tracer.start()

    sleep(5.0)

    dump_tracer(robot.tracer)

    # --- DISPLAY
    zmp_data = np.loadtxt('/tmp/dg_' + robot.dynamic.name + '-zmp.dat')
    zmpDes_data = np.loadtxt('/tmp/dg_' + robot.estimator.name + '-dcm.dat')

    plt.figure()
    plt.plot(zmp_data[:, 1], 'b-')
    plt.plot(zmpDes_data[:, 1], 'b--')
    plt.plot(zmp_data[:, 2], 'r-')
    plt.plot(zmpDes_data[:, 2], 'r--')
    plt.title('ZMP real vs desired')
    plt.legend(['Real x', 'Desired x', 'Real y', 'Desired y'])

    plt.figure()
    plt.plot(zmp_data[:, 1] - zmpDes_data[:, 1], 'b-')
    plt.plot(zmp_data[:, 2] - zmpDes_data[:, 2], 'r-')
    plt.title('ZMP error')
    plt.legend(['Error on x', 'Error on y'])

    plt.show()

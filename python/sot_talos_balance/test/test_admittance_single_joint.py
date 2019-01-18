from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

run_test('appli_admittance_single_joint.py')

sleep(1.0)
runCommandClient('robot.admittance_control.tauDes.value = target')
runCommandClient('robot.admittance_control.setPosition(robot.device.state.value)')
runCommandClient('robot.sot.push(robot.taskJoint.task.name)')

sleep(5.0)
des_tau = evalCommandClient('target[QJOINT]')
tau = evalCommandClient('robot.device.ptorque.value[JOINT]')

print("Desired torque: %f" % des_tau)
print("Current torque: %f" % tau)


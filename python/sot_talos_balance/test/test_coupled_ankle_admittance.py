from time import sleep

from sot_talos_balance.utils.run_test_utils import evalCommandClient, run_test

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_coupled_ankle_admittance.py')

sleep(1.0)

RightPitchJoint = 10
LeftPitchJoint = 4
RightRollJoint = 11
LeftRollJoint = 5

input("Wait before evaluation")

tauRP = evalCommandClient('robot.device.ptorque.value[RightPitchJoint]')
tauLP = evalCommandClient('robot.device.ptorque.value[LeftPitchJoint]')
tauRR = evalCommandClient('robot.device.ptorque.value[RightRollJoint]')
tauLR = evalCommandClient('robot.device.ptorque.value[LeftRollJoint]')

tauDesRP = evalCommandClient('robot.pitchController.tauDesR.value')
tauDesLP = evalCommandClient('robot.pitchController.tauDesL.value')
tauDesRR = evalCommandClient('robot.rollController.tauDesR.value')
tauDesLR = evalCommandClient('robot.rollController.tauDesL.value')

print("Desired torques: %f, %f, %f, %f" % (tauDesRP[0], tauDesLP[0], tauDesRR[0], tauDesLR[0]))
print("Current torques: %f, %f, %f, %f" % (tauRP, tauLP, tauRR, tauLR))

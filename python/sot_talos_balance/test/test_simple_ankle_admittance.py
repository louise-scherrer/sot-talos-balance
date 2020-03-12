from time import sleep

from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, evalCommandClient, run_test, runCommandClient

run_test('appli_simple_ankle_admittance.py')

sleep(1.0)

RightPitchJoint = 10
LeftPitchJoint = 4
RightRollJoint = 11
LeftRollJoint = 5

c = ask_for_confirmation("Do you want to use the current torques values as reference?")
if c:
    runCommandClient('robot.rightPitchAnkleController.tauDes.value = robot.device.ptorque.value[RightPitchJoint]')
    runCommandClient('robot.leftPitchAnkleController.tauDes.value = robot.device.ptorque.value[LeftPitchJoint]')
    runCommandClient('robot.rightRollAnkleController.tauDes.value = robot.device.ptorque.value[RightRollJoint]')
    runCommandClient('robot.leftRollAnkleController.tauDes.value = robot.device.ptorque.value[LeftRollJoint]')

    print("Setting desired torques with current values")
else:
    c2 = ask_for_confirmation("Do you want to use nul torques values as reference?")
    if c2:
        runCommandClient('robot.rightPitchAnkleController.tauDes.value = [0.0]')
        runCommandClient('robot.leftPitchAnkleController.tauDes.value = [0.0]')
        runCommandClient('robot.rightRollAnkleController.tauDes.value = [0.0]')
        runCommandClient('robot.leftRollAnkleController.tauDes.value = [0.0]')

        print("Setting desired torques with nul values")
    else:
        runCommandClient('robot.rightPitchAnkleController.tauDes.value = [-2.5164434873140853]')
        runCommandClient('robot.leftPitchAnkleController.tauDes.value = [-2.5164434873140853]')
        runCommandClient('robot.rightRollAnkleController.tauDes.value = [0.0]')
        runCommandClient('robot.leftRollAnkleController.tauDes.value = [0.0]')

        print("Setting desired torques with implemented values")

print("Pushing the ankle tasks...")

runCommandClient('robot.sot.push(robot.rightAnklePitchTask.task.name)')
runCommandClient('robot.sot.push(robot.leftAnklePitchTask.task.name)')
runCommandClient('robot.sot.push(robot.rightAnkleRollTask.task.name)')
runCommandClient('robot.sot.push(robot.leftAnkleRollTask.task.name)')

sleep(5.0)
tauRP = evalCommandClient('robot.device.ptorque.value[RightPitchJoint]')
tauLP = evalCommandClient('robot.device.ptorque.value[LeftPitchJoint]')
tauRR = evalCommandClient('robot.device.ptorque.value[RightRollJoint]')
tauLR = evalCommandClient('robot.device.ptorque.value[LeftRollJoint]')

tauDesRP = evalCommandClient('robot.rightPitchAnkleController.tauDes.value')
tauDesLP = evalCommandClient('robot.leftPitchAnkleController.tauDes.value')
tauDesRR = evalCommandClient('robot.rightRollAnkleController.tauDes.value')
tauDesLR = evalCommandClient('robot.leftRollAnkleController.tauDes.value')

print("Desired torques: %f, %f, %f, %f" % (tauDesRP[0], tauDesLP[0], tauDesRR[0], tauDesLR[0]))
print("Current torques: %f, %f, %f, %f" % (tauRP, tauLP, tauRR, tauLR))

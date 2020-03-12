from time import sleep

from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient

run_test('appli_COMTraj.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,2.0)')

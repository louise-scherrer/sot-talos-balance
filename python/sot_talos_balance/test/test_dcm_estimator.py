from time import sleep

from sot_talos_balance.utils.run_test_utils import run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_dcm_estimator.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# plug ZMP emergency signal
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
sleep(20.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,2.0)')
sleep(20.0)

input("Wait before leaving the simulation")

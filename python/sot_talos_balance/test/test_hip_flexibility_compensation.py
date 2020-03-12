from time import sleep

from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_hip_flexibility_compensation.py')

sleep(10.0)

input("Wait before plugging the hip calibration")
runCommandClient('plug(robot.hipComp.q_cmd, robot.device.control)')

sleep(10.0)
input("Wait before leaving the simulation")

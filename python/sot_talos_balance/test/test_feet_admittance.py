'''Test feet admittance control'''
from sot_talos_balance.utils.run_test_utils import run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_feet_admittance.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

print('Set saturation value')
runCommandClient('robot.admBF_dqSaturation.sin.value = [0.0, 0.0, 0.01, 0.0, 0.0, 0.0]')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

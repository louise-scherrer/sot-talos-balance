'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sys import argv

from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

test_folder = argv[1] if len(argv) > 1 else 'TestKajita2003WalkingOnSpot64/DSP20SSP780'
print('Using folder ' + test_folder)

runCommandClient('test_folder = "' + test_folder + '"')

run_test('appli_dcmZmpControl_file.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerTrajGen.sin.value = 1')
else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

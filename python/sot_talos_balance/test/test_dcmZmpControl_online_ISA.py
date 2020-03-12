from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_dcmZmpControl_online.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

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
    runCommandClient('robot.triggerPG.sin.value = 1')
else:
    print('Not executing the trajectory')

raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')


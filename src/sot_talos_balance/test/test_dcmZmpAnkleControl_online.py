# Implementation test on the ankle controller admittance of Stephane Caron based on dcmZmpControl_online simulation

from sys import argv
from sot_talos_balance.utils.run_test_utils import *
from time import sleep
from dynamic_graph import * # for entity graph display

run_test('appli_dcmZmpAnkleControl_online.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')

runCommandClient('plug(robot.wp.zmpDes,robot.wrenchDistributor.zmpDes)')

runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')

runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')


sleep(4.0)

runCommandClient('robot.leftAnkleController.setState(robot.wp.footLeftDes.value)') # new
runCommandClient('robot.rightAnkleController.setState(robot.wp.footRightDes.value)') # new
runCommandClient('robot.rightAnkleController.gainsXY.value = GainsXY')
runCommandClient('robot.leftAnkleController.gainsXY.value = GainsXY') # ou 0.1
c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerPG.sin.value = 1')
    runCommandClient('robot.pg.velocitydes.value = (0.1,0.0,0.0)')
    sleep(15)
    runCommandClient('robot.pg.velocitydes.value = (0.0,0.0,0.0)')
    sleep(10)


else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')


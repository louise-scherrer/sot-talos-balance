'''Test for online generation of movements for Talos using M. Naveau's pattern generator. Velocity command via signal robot.pg.velocitydes.value = (Vx, Vy, Omega_z)'''

## File adapted from appli_dcmZmpControl_file.py and appli_dcmZmpControl_online_ISA.py written by Isabelle Maroger

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
#runCommandClient('plug(robot.wrenchDistributor.emergencyStop,robot.cm.emergencyStop_distribute)')

#runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')

#runCommandClient('plug(robot.wrenchDistributor.zmpRef,robot.com_admittance_control.zmpDes)')
# 26.06 test
runCommandClient('plug(robot.dcm_control.zmpRef,robot.wrenchDistributor.zmpDes)')
#runCommandClient('plug(robot.wp.zmpDes,robot.wrenchDistributor.zmpDes)')

runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')

#24.07 test ici
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')
# fin test

sleep(4.0)

runCommandClient('robot.leftAnkleController.setState(robot.wp.footLeftDes.value)') # new
runCommandClient('robot.rightAnkleController.setState(robot.wp.footRightDes.value)') # new
runCommandClient('robot.rightAnkleController.gainsXY.value = GainsXY')
runCommandClient('robot.leftAnkleController.gainsXY.value = GainsXY') # ou 0.1

# test remonter ca 24.07
#runCommandClient('robot.dcm_control.resetDcmIntegralError()')
#runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

#test
#runCommandClient('plug(robot.wrenchDistributor.copRight, controller.pRef)')
#runCommandClient('plug(robot.wrenchDistributor.copLeft, controller.pRef)')



c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerPG.sin.value = 1')
#    writeGraph('/local/lscherrer/lscherrer/Scripts/Results/my_dyn_graph.dot')

else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')


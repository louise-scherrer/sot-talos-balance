'''Test CoM admittance control as described in paper, with a Reference Velocity provided'''

## File adapted from appli_dcmZmpControl_file.py and appli_dcmZmpControl_online_ISA.py written by Isabelle Maroger
## ALL THINGS ADDED OR CHANGED ARE COMMENTED IN CAPITAL LETTERS

from sys import argv

#from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, run_ft_calibration, run_test, runCommandClient
## INSTEAD, ISA WROTE (CHECK DIFFERENCES)
from sot_talos_balance.utils.run_test_utils import *
from time import sleep
## END DIFFERENCE

from dynamic_graph import * # for entity graph display

## NEXT FEW LINES IRRELEVENT AS WISH TO GO ONLINE --> COMMENTED
#try:
#    # Python 2
#    input = raw_input  # noqa
#except NameError:
#    pass
#
#test_folder = argv[1] if len(argv) > 1 else 'TestKajita2003WalkingOnSpot64/DSP20SSP780'
#print('Using folder ' + test_folder)
#
#runCommandClient('test_folder = "' + test_folder + '"')
## END DIFFERENCE

run_test('appli_joystick_dcmZmpControl_file.py')

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
#    runCommandClient('robot.triggerTrajGen.sin.value = 1') # NAMED DIFFERENTLY IN ISA'S SCRIPT, SEE BELOW
    runCommandClient('robot.triggerPG.sin.value = 1')
    writeGraph('/local/lscherrer/lscherrer/Scripts/Results/my_dyn_graph.dot')

else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

## ADDED ;)
print('Bye!')


'''Test CoM admittance control as described in paper, with pre-loaded movements'''

## File adapted from appli_dcmZmpControl_file.py and appli_dcmZmpControl_online_ISA.py written by Isabelle Maroger
## ALL THINGS ADDED OR CHANGED ARE COMMENTED IN CAPITAL LETTERS

from sys import argv
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

#from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, run_ft_calibration, run_test, runCommandClient
## INSTEAD, ISA WROTE (CHECK DIFFERENCES)
from sot_talos_balance.utils.run_test_utils import *
from time import sleep
## END DIFFERENCE

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

## ESSAI DE LISTENER 1/2
#def callback(data):
#    print('appel callback, j ai compris : ')
#    print(data)
#    robot.pg.velocitydes.value = data

#def teleop_listener():
#    rospy.init_node('teleop_listener', anonymous = True)
#    rospy.Subscriber("teleop_order", Vector3, callback)
## FIN

run_test('appli_WIP_joystick.py')

run_ft_calibration('robot.ftc')

current_velocity_order = (0.0,0.0,0.0)

#25.03 Test de mettre subscriber et callback directement ici
def callback(data):
#    print('appel callback, j ai compris ca comme donnee: ') # attention empeche de voire passer les 'wait before executing trajectory'
#    print(data)
    new_velocity_order = (data.linear.x, data.linear.y, data.linear.z)
    global current_velocity_order
    if (new_velocity_order != current_velocity_order):
        current_velocity_order = new_velocity_order
        to_send_order = "robot.pg.velocitydes.value = (" + str(data.linear.x) + "," + str(data.linear.y) + "," + str(data.linear.z) + ")"
        print(to_send_order)
        runCommandClient(to_send_order)

#    runCommandClient('robot.pg.velocitydes.value = velocity_data') # faire les modifs pour que cmd_vel soit un vecteur ou data.linear.x,y,z ?
#sans runCommandClient, ne connait pas robot, avec tel quel connait pas data
rospy.init_node('teleop_listener', anonymous = True) # je suis deja dans un noeud, non ?
rospy.Subscriber("turtlebot_teleop/cmd_vel", Twist, callback) # passer a Vector3 pour le type de message
#fin ajout


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

else:
    print('Not executing the trajectory')

## ESSAI DE LISTENER 2/2
#teleop_listener()
#print('a saute')
## FIN

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

## ADDED ;)
print('Bye!')


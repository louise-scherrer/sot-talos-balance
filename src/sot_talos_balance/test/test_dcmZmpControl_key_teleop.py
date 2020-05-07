'''Test with keyboard teleoperation of Talos using script keyboard_teleop.py for keyboard control and appli_dcmZmpControl_online.py'''

## File adapted from appli_dcmZmpControl_file.py and appli_dcmZmpControl_online_ISA.py written by Isabelle Maroger

from sys import argv
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_dcmZmpControl_online.py')

run_ft_calibration('robot.ftc')

current_velocity_order = (0.0,0.0,0.0)


def callback(data):
#    print('calling callback, I understood this data: ') # caution makes it harder to start the simulation, floods these messages: 'wait before executing trajectory'
#    print(data)
    new_velocity_order = (data.linear.x, data.linear.y, data.angular.z)
    global current_velocity_order
    if (new_velocity_order != current_velocity_order):
        current_velocity_order = new_velocity_order
        to_send_order = "robot.pg.velocitydes.value = (" + str(data.linear.x) + "," + str(data.linear.y) + "," + str(data.angular.z) + ")"
        print(to_send_order)
        runCommandClient(to_send_order)

rospy.init_node('teleop_listener', anonymous = True)
rospy.Subscriber("pyrene_teleop/cmd_vel", Twist, callback)

input("Wait before running the test")

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

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')


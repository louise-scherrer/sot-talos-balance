'''Test CoM admittance control as described in paper'''
from sys import argv
from time import sleep

from sot_talos_balance.utils.run_test_utils import (ask_for_confirmation, get_file_folder, run_ft_calibration,
                                                    run_test, runCommandClient)

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

test_folder, sot_talos_balance_folder = get_file_folder(argv)

run_test('appli_dcm_zmp_control.py')

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
runCommandClient('robot.dcm_control.Kz.value = Kz_dcm')

if test_folder is not None:
    c = ask_for_confirmation('Execute trajectory?')
    if c:
        print('Executing the trajectory')
        runCommandClient('robot.triggerTrajGen.sin.value = 1')
    else:
        print('Not executing the trajectory')
else:
    c = ask_for_confirmation("Execute a sinusoid?")
    if c:
        print("Putting the robot in position...")
        runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
        sleep(1.0)
        print("Robot is in position!")

        c2 = ask_for_confirmation("Confirm executing the sinusoid?")
        if c2:
            print("Executing the sinusoid...")
            runCommandClient('robot.comTrajGen.startSinusoid(1,0.025,2.0)')
            print("Sinusoid started!")
        else:
            print("Not executing the sinusoid")

        c3 = ask_for_confirmation("Put the robot back?")
        if c3:
            print("Stopping the robot...")
            runCommandClient('robot.comTrajGen.stop(1)')
            sleep(5.0)
            print("Putting the robot back...")
            runCommandClient('robot.comTrajGen.move(1,0.0,1.0)')
            sleep(1.0)
            print("The robot is back in position!")
        else:
            print("Not putting the robot back")
    else:
        print("Not executing the sinusoid")

    c = ask_for_confirmation("Raise the foot?")
    if c:
        print("Putting the robot in position...")
        runCommandClient('robot.comTrajGen.move(1,-0.08,10.0)')
        sleep(10.0)
        print("Robot is in position!")

        foot_on_ground = True

        c2 = ask_for_confirmation("Confirm raising the foot?")
        if c2:
            print("Raising the foot...")
            runCommandClient('h = robot.dynamic.LF.value[2][3]')
            runCommandClient('robot.lfTrajGen.move(2,h+0.05,10.0)')
            sleep(10.0)
            print("Foot has been raised!")
            foot_on_ground = False
            c3 = ask_for_confirmation("Put the foot back?")
            if c3:
                print("Putting the foot back...")
                runCommandClient('robot.lfTrajGen.move(2,h,10.0)')
                sleep(10.0)
                print("The foot is back in position!")
                foot_on_ground = True
            else:
                print("Not putting the foot back")
        else:
            print("Not raising the foot")

        if foot_on_ground:
            c4 = ask_for_confirmation("Put the robot back?")
            if c4:
                print("Putting the robot back...")
                runCommandClient('robot.comTrajGen.move(1,0.0,10.0)')
                sleep(10.0)
                print("The robot is back in position!")
            else:
                print("Not putting the robot back")
    else:
        print("Not raising the foot")

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')

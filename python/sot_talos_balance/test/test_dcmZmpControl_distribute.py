'''Test CoM admittance control as described in paper.'''
from time import sleep

from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_dcmZmpControl_distribute.py')

run_ft_calibration('robot.ftc')

use_force_distribution = ask_for_confirmation("Use output of force distribution?")
if use_force_distribution:
    print("Using output of force distribution")
else:
    print("Not using output of force distribution")

input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
if use_force_distribution:
    runCommandClient('plug(robot.distribute.emergencyStop,robot.cm.emergencyStop_distribute)')
    runCommandClient('plug(robot.distribute.zmpRef,robot.com_admittance_control.zmpDes)')
else:
    runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

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
    runCommandClient('robot.rhoTrajGen.move(0,0.4,10.0)')
    sleep(10.0)
    print("Robot is in position!")

    c2 = ask_for_confirmation("Confirm raising the foot?")
    if c2:
        print("Raising the foot...")
        runCommandClient('robot.phaseTrajGen.set(0,-1)')
        runCommandClient('h = robot.dynamic.LF.value[2][3]')
        runCommandClient('robot.lfTrajGen.move(2,h+0.05,10.0)')
        sleep(10.0)
        print("Foot has been raised!")
        c3 = ask_for_confirmation("Put the foot back?")
    else:
        print("Not raising the foot")
        c3 = False

    if c3:
        print("Putting the foot back...")
        runCommandClient('robot.lfTrajGen.move(2,h,10.0)')
        sleep(10.0)
        runCommandClient('robot.phaseTrajGen.set(0,0)')
        print("The foot is back in position!")
        c4 = ask_for_confirmation("Put the robot back?")
    else:
        print("Not putting the foot back")
        c4 = False

    if c4:
        print("Putting the robot back...")
        runCommandClient('robot.comTrajGen.move(1,0.0,10.0)')
        runCommandClient('robot.rhoTrajGen.move(0,0.5,10.0)')
        sleep(10.0)
        print("The robot is back in position!")
else:
    print("Not raising the foot")

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

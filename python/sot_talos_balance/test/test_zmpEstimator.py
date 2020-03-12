from time import sleep

import matplotlib.pyplot as plt
import numpy as np

from sot_talos_balance.utils.run_test_utils import evalCommandClient, run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_zmpEstimator.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# plug ZMP emergency signal
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
sleep(20.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,2.0)')
sleep(20.0)

runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
zmpEst_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.zmp_estimator.name') + '-zmp.dat')
zmpDyn_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-zmp.dat')
com_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
forceRLEG_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.device.name') + '-forceRLEG.dat')
forceLLEG_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.device.name') + '-forceLLEG.dat')

plt.ion()

plt.figure()
plt.plot(zmpEst_data[:, 1], 'b-')
plt.plot(zmpDyn_data[:, 1], 'b--')
plt.plot(com_data[:, 1], 'b:')
plt.plot(zmpEst_data[:, 2], 'r-')
plt.plot(zmpDyn_data[:, 2], 'r--')
plt.plot(com_data[:, 2], 'r:')
plt.title('ZMP estimate vs dynamic vs CoM (planar)')
plt.legend(['x estimate', 'x dynamic', 'x CoM', 'y estimate', 'y dynamic', 'y CoM'])

plt.figure()
plt.plot(com_data[:, 1], 'b-')
plt.plot(com_data[:, 2], 'r-')
plt.plot(com_data[:, 3], 'g-')
plt.title('COM')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(zmpDyn_data[:, 1], 'b-')
plt.plot(zmpDyn_data[:, 2], 'r-')
plt.plot(zmpDyn_data[:, 3], 'g-')
plt.title('ZMP dynamic')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(zmpEst_data[:, 1], 'b-')
plt.plot(zmpEst_data[:, 2], 'r-')
plt.plot(zmpEst_data[:, 3], 'g-')
plt.title('ZMP estimate')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(forceLLEG_data[:, 1], 'b-')
plt.plot(forceLLEG_data[:, 2], 'r-')
plt.plot(forceLLEG_data[:, 3], 'g-')
plt.plot(forceLLEG_data[:, 4], 'b--')
plt.plot(forceLLEG_data[:, 5], 'r--')
plt.plot(forceLLEG_data[:, 6], 'g--')
plt.title('forceLLEG')
plt.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

plt.figure()
plt.plot(forceRLEG_data[:, 1], 'b-')
plt.plot(forceRLEG_data[:, 2], 'r-')
plt.plot(forceRLEG_data[:, 3], 'g-')
plt.plot(forceRLEG_data[:, 4], 'b--')
plt.plot(forceRLEG_data[:, 5], 'r--')
plt.plot(forceRLEG_data[:, 6], 'g--')
plt.title('forceRLEG')
plt.legend(['fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

input("Wait before leaving the simulation")

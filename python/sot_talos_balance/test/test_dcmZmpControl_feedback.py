'''Test CoM admittance control as described in paper.'''
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

from sot_talos_balance.utils.run_test_utils import evalCommandClient, run_ft_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_dcmZmpControl_feedback.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(comDes,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

sleep(30.0)

runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
comDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') +
                         '-dcmDes.dat')  # desired CoM (workaround)
comEst_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.cdc_estimator.name') +
                         '-c.dat')  # estimated CoM (not directly employed)
comRef_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.com_admittance_control.name') +
                         '-comRef.dat')  # reference CoM
comSOT_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')  # resulting SOT CoM

dcmDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') + '-dcmDes.dat')  # desired DCM
dcmEst_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.estimator.name') + '-dcm.dat')  # estimated DCM

zmpDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') + '-zmpDes.dat')  # desired ZMP
zmpSOT_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-zmp.dat')  # SOT ZMP
zmpEst_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.zmp_estimator.name') + '-zmp.dat')  # estimated ZMP
zmpRef_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') + '-zmpRef.dat')  # reference ZMP

plt.ion()

plt.figure()
plt.plot(comDes_data[:, 1], 'b--')
plt.plot(comEst_data[:, 1], 'b-')
plt.plot(comRef_data[:, 1], 'b:')
plt.plot(comSOT_data[:, 1], 'b-.')
plt.plot(comDes_data[:, 2], 'r--')
plt.plot(comEst_data[:, 2], 'r-')
plt.plot(comRef_data[:, 2], 'r:')
plt.plot(comSOT_data[:, 2], 'r-.')
plt.plot(comDes_data[:, 3], 'g--')
plt.plot(comEst_data[:, 3], 'g-')
plt.plot(comRef_data[:, 3], 'g:')
plt.plot(comSOT_data[:, 3], 'g-.')
plt.title('COM')
plt.legend([
    'Desired x', 'Estimated x', 'Reference x', 'SOT x', 'Desired y', 'Estimated y', 'Reference y', 'SOT y',
    'Desired z', 'Estimated z', 'Reference z', 'SOT z'
])

plt.figure()
plt.plot(dcmDes_data[:, 1], 'b--')
plt.plot(dcmEst_data[:, 1], 'b-')
plt.plot(dcmDes_data[:, 2], 'r--')
plt.plot(dcmEst_data[:, 2], 'r-')
plt.title('DCM')
plt.legend(['Desired x', 'Estimated x', 'Desired y', 'Estimated y'])

plt.figure()
plt.plot(zmpDes_data[:, 1], 'b--')
plt.plot(zmpSOT_data[:, 1], 'b-')
plt.plot(zmpRef_data[:, 1], 'b:')
plt.plot(zmpDes_data[:, 2], 'r--')
plt.plot(zmpSOT_data[:, 2], 'r-')
plt.plot(zmpRef_data[:, 2], 'r:')
plt.title('ZMP')
plt.legend(['Desired x', 'SOT x', 'Reference x', 'Desired y', 'SOT y', 'Reference y'])

zmpErrSOT = zmpSOT_data - zmpDes_data

plt.figure()
plt.plot(zmpErrSOT[:, 1], 'b-')
plt.plot(zmpErrSOT[:, 2], 'r-')
plt.title('ZMP SOT error')
plt.legend(['Error on x', 'Error on y'])

input("Wait before leaving the simulation")

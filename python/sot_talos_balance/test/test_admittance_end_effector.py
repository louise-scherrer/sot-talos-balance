from time import sleep

from sot_talos_balance.utils.run_test_utils import run_ft_wrist_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_admittance_end_effector.py')

run_ft_wrist_calibration('robot.forceCalibrator')

input("Wait before running the test")

sleep(10.0)

# --- DISPLAY
# force_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.controller.name') + '-force.dat')
# w_force_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.controller.name') + '-w_force.dat')

# plt.ion()

# plt.figure()
# plt.plot(force_data[:,1],'b-')
# plt.plot(force_data[:,2],'r-')
# plt.plot(force_data[:,3],'g-')
# plt.title('force')
# plt.legend(['force x', 'force y', 'force z'])

# plt.figure()
# plt.plot(w_force_data[:,1],'b-')
# plt.plot(w_force_data[:,2],'r-')
# plt.plot(w_force_data[:,3],'g-')
# plt.title('w_force')
# plt.legend(['w_force x', 'w_force y', 'w_force z'])

input("Wait before leaving the simulation")

runCommandClient('dump_tracer(robot.tracer)')

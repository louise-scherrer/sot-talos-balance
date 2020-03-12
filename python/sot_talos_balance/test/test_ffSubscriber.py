from time import sleep

import matplotlib.pyplot as plt
import numpy as np

from sot_talos_balance.utils.gazebo_utils import GazeboLinkStatePublisher
from sot_talos_balance.utils.run_test_utils import evalCommandClient, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

pub = GazeboLinkStatePublisher('base_link', 1000)
print("Starting Gazebo link state publisher...")
pub.start()
print("Gazebo link state publisher started")
input("Wait before running the test")

run_test('appli_ffSubscriber.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')
sleep(5.0)
runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
com_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
pos_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.subscriber.name') + '-position.dat')
vel_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.subscriber.name') + '-velocity.dat')

plt.ion()

plt.figure()
plt.plot(com_data[:, 1], 'b-')
plt.plot(com_data[:, 2], 'r-')
plt.plot(com_data[:, 3], 'g-')
plt.title('COM')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(pos_data[:, 1], 'b-')
plt.plot(pos_data[:, 2], 'r-')
plt.plot(pos_data[:, 3], 'g-')
plt.title('Position measure')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(pos_data[:, 4], 'b-')
plt.plot(pos_data[:, 5], 'r-')
plt.plot(pos_data[:, 6], 'g-')
plt.title('Orientation measure')
plt.legend(['yaw', 'pitch', 'roll'])

plt.figure()
plt.plot(vel_data[:, 1], 'b-')
plt.plot(vel_data[:, 2], 'r-')
plt.plot(vel_data[:, 3], 'g-')
plt.title('Linear velocity measure')
plt.legend(['x', 'y', 'z'])

plt.figure()
plt.plot(vel_data[:, 4], 'b-')
plt.plot(vel_data[:, 5], 'r-')
plt.plot(vel_data[:, 6], 'g-')
plt.title('Angular velocity measure')
plt.legend(['x', 'y', 'z'])

input("Wait before leaving the simulation")
print("Stopping Gazebo link state publisher...")
pub.stop()
sleep(0.1)
print("Gazebo link state publisher stopped")

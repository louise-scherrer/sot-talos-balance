import matplotlib.pyplot as plt
import numpy as np

folder = 'ros/sot_talos_balance/data/TestKajita2003WalkingOnSpot64/DSP20SSP780/'

LF = np.loadtxt(folder + 'LeftFoot.dat')
RF = np.loadtxt(folder + 'RightFoot.dat')
phase = np.loadtxt(folder + 'Phase.dat')
rho = np.loadtxt(folder + 'Rho.dat')

plt.plot(LF[:, 2])
plt.plot(RF[:, 2])
scale = 0.1
bias = LF[0, 2]
plt.plot(phase[:, 0] * scale + bias)
plt.plot(rho[:, 0])
plt.legend(['left', 'right', 'phase', 'rho'])
leg = plt.legend(['left', 'right', 'phase', 'rho'])
if leg:
    leg.draggable()

plt.show()

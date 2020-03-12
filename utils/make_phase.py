import matplotlib.pyplot as plt
import numpy as np

folder = 'ros/sot_talos_balance/data/TestKajita2003WalkingOnSpot64/DSP20SSP780/'

LF = np.loadtxt(folder + 'LeftFoot.dat')
RF = np.loadtxt(folder + 'RightFoot.dat')

T = LF.shape[0]
phase = np.zeros([T, 3])
rho = np.zeros([T, 3])

rho_m = 0.4

ts = 3214
DSP = 20
SSP = 780
LP = DSP / 2
UP = LP
steps = 16
right = True

t0 = ts - LP
rho_M = 1 - rho_m


def min_jerk(x_init, x_final, t, t_max):
    td = float(t) / float(t_max)
    td2 = td * td
    td3 = td2 * td
    td4 = td3 * td
    td5 = td4 * td
    p = 10. * td3 - 15 * td4 + 6 * td5
    x = x_init + (x_final - x_init) * p

    dp = (30 * td2 - 60 * td3 + 30 * td4) / t_max
    dx = (x_final - x_init) * dp

    ddp = (60 * td - 180 * td2 + 120 * td3) / (t_max * t_max)
    ddx = (x_final - x_init) * ddp

    return np.matrix([x, dx, ddx]).T


if right:
    rho_1 = rho_m
    rho_2 = rho_M
    phase_1 = -1
    phase_2 = 1
else:
    rho_1 = rho_M
    rho_2 = rho_m
    phase_1 = 1
    phase_2 = -1

t = 0
for i in range(t0):
    rho[t, 0] = 0.5
    t += 1

for i in range(LP):
    rho[t, :] = min_jerk(0.5, rho_1, i, LP).T
    t += 1

for s in range(steps):
    for i in range(SSP):
        rho[t, 0] = rho_1
        phase[t, 0] = phase_1
        t += 1
    if s == (steps - 1):
        rho_end = 0.5
        EP = UP
    else:
        rho_end = rho_2
        EP = DSP
    for i in range(EP):
        rho[t, :] = min_jerk(rho_1, rho_end, i, EP).T
        t += 1
    rho_1, rho_2 = (rho_2, rho_1)
    phase_1, phase_2 = (phase_2, phase_1)

for i in range(t, T):
    rho[i, 0] = 0.5

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

np.savetxt(folder + 'Phase.dat', phase, fmt='%d')
np.savetxt(folder + 'Rho.dat', rho)

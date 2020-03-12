import matplotlib.pyplot as plt

w = 3.
A = 8.
N = 100000
a = 1
Kp = 8.
w2 = w * w
dt = 0.0001
# initial conditions
c = 1.
dc = 0.
ddc = 10
log_c = []
z = c - 1 / w2 * ddc
z_m = z
for i in range(N):
    # measure and filter
    z = c - 1 / w2 * ddc
    z_m = z * a + (1 - a) * z_m
    # control
    xi = c + dc / w
    ddc = A * (z_m) - A * (1 + Kp / w) * xi
    # integrate
    dc += dt * ddc
    c += dt * dc
    log_c.append(c)
    # print c

plt.plot(log_c)
plt.show()

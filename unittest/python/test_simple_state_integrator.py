from __future__ import print_function

import numpy as np
from numpy.testing import assert_almost_equal as assertApprox

from sot_talos_balance.simple_state_integrator import SimpleStateIntegrator

dt = 1e-3
initstate = [0.]*9

integrator = SimpleStateIntegrator("integrator")
integrator.init(dt)
integrator.setState(initstate)
integrator.setVelocity([0.]*9)
t = 0

integrator.control.value = [0.]*9
integrator.state.recompute(0)
assertApprox(integrator.state.value, initstate)

t += 1
integrator.control.value = [1., 0., 0.] + [0.]*6
integrator.state.recompute(t)
assertApprox(integrator.state.value, [dt, 0., 0.] + [0.]*6)

t += 1
integrator.control.value = [0., 1., 0.] + [0.]*6
integrator.state.recompute(t)
assertApprox(integrator.state.value, [dt, dt, 0.] + [0.]*6)

t += 1
integrator.control.value = [0., 0., 1.] + [0.]*6
integrator.state.recompute(t)
assertApprox(integrator.state.value, [dt, dt, dt] + [0.]*6)

t += 1
integrator.control.value = [0.]*3 + [1., 0., 0.] + [0.]*3
integrator.state.recompute(t)
assertApprox(integrator.state.value, [dt]*3 + [dt, 0., 0.] + [0.]*3)

t += 1
integrator.setState(initstate)
integrator.control.value = [0.]*3 + [0., 1., 0.] + [0.]*3
integrator.state.recompute(t)
assertApprox(integrator.state.value, [0.]*3 + [0., dt, 0.] + [0.]*3)

t += 1
integrator.setState(initstate)
integrator.control.value = [0.]*3 + [0., 0., 1.] + [0.]*3
integrator.state.recompute(t)
assertApprox(integrator.state.value, [0.]*3 + [0., 0., dt] + [0.]*3)

t += 1
integrator.setState(initstate)
integrator.control.value = [0.]*6 + [1., 0., 0.]
integrator.state.recompute(t)
assertApprox(integrator.state.value, [0.]*6 + [dt, 0., 0.])

integrator.setState(initstate)
N = 10
for k in range(N):
    t += 1
    integrator.control.value = [0.]*3 + [1., 0., 0.] + [0.]*3
    integrator.state.recompute(t)
    assertApprox(integrator.state.value, [0.]*3 + [dt*(k+1), 0., 0.] + [0.]*3)

t+=1
integrator.setState(initstate + [0.])
integrator.control.value = [0.]*3 + [1., 0., 0.] + [0.]*3
try:
  integrator.state.recompute(t)
except:
  pass
else:
  raise AssertionError("No exception raised for mismatching state and control size")

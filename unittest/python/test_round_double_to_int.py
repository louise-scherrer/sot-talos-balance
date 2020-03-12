from __future__ import print_function

import numpy as np

from sot_talos_balance.round_double_to_int import RoundDoubleToInt

rd = RoundDoubleToInt('round')
time = 0

rd.sin.value = 0.
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, 0)
time += 1

rd.sin.value = 1.
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, 1)
time += 1

rd.sin.value = 1.3
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, 1)
time += 1

rd.sin.value = 1.8
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, 2)
time += 1

rd.sin.value = 2.5
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, 3)
time += 1

rd.sin.value = -1.
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, -1)
time += 1

rd.sin.value = -1.3
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, -1)
time += 1

rd.sin.value = -1.8
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, -2)
time += 1

rd.sin.value = -2.5
rd.sout.recompute(time)
print("%f -> %d" % (rd.sin.value, rd.sout.value))
np.testing.assert_equal(rd.sout.value, -3)
time += 1

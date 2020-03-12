from __future__ import print_function

import numpy as np
from numpy.testing import assert_almost_equal as assertApprox

from sot_talos_balance.euler_to_quat import EulerToQuat
from sot_talos_balance.pose_quaternion_to_matrix_homo import PoseQuaternionToMatrixHomo
from sot_talos_balance.quat_to_euler import QuatToEuler

# --- Euler to quat ---
print("--- Euler to quat ---")

signal_in = [0.0, 0.0, 0.5, 0.0, 0.0, np.pi, 0.2, 0.6]
e2q = EulerToQuat('e2q')
e2q.euler.value = signal_in
print(e2q.euler.value)
e2q.quaternion.recompute(0)
print(e2q.quaternion.value)

assertApprox(e2q.quaternion.value, [0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.2, 0.6], 6)

# --- Quat to Euler ---
print("--- Quat to Euler ---")

signal_in = [0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.2, 0.6]
q2e = QuatToEuler('q2e')
q2e.quaternion.value = signal_in
print(q2e.quaternion.value)
q2e.euler.recompute(0)
print(q2e.euler.value)

assertApprox(q2e.euler.value, [0.0, 0.0, 0.5, 0.0, 0.0, np.pi, 0.2, 0.6], 6)

# --- Quat to homogeneous ---
print("--- Quat to homogeneous ---")

signal_in = [0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0]
q2m = PoseQuaternionToMatrixHomo('q2m')
q2m.sin.value = signal_in
print(q2m.sin.value)
q2m.sout.recompute(0)
print(q2m.sout.value)

expected = ((-1.0, 0.0, 0.0, 0.0), (0.0, -1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.5), (0.0, 0.0, 0.0, 1.0))
assertApprox(q2m.sout.value, expected, 6)

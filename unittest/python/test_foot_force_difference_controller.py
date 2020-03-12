from __future__ import print_function

import numpy as np
from numpy.testing import assert_almost_equal as assertApprox

from sot_talos_balance.foot_force_difference_controller import FootForceDifferenceController

controller = FootForceDifferenceController("footController")
controller.init()

controller.dfzAdmittance.value = 1.
controller.vdcFrequency.value = 0.
controller.vdcDamping.value = 0.

gainSwing = 1.
gainStance = 2.
gainDouble = 3.

controller.wrenchRight.value = [0.] * 2 + [500.] + [0.] * 3
controller.wrenchLeft.value = [0.] * 2 + [300.] + [0.] * 3
controller.wrenchRightDes.value = [0.] * 2 + [400.] + [0.] * 3
controller.wrenchLeftDes.value = [0.] * 2 + [400.] + [0.] * 3

controller.gainSwing.value = gainSwing
controller.gainStance.value = gainStance
controller.gainDouble.value = gainDouble

print("---- Input ----")
print("wrenchRight:    %s" % str(controller.wrenchRight.value))
print("wrenchLeft:     %s" % str(controller.wrenchLeft.value))
print("wrenchRightDes: %s" % str(controller.wrenchRightDes.value))
print("wrenchLeftDes:  %s" % str(controller.wrenchLeftDes.value))
print()

print("gainSwing:  %s" % str(controller.gainSwing.value))
print("gainStance: %s" % str(controller.gainStance.value))
print("gainDouble: %s" % str(controller.gainDouble.value))
print()

print("---- Double support ----")

controller.phase.value = 0

controller.posRightDes.value = np.eye(4).tolist()
controller.posLeftDes.value = np.eye(4).tolist()
controller.posRight.value = np.eye(4).tolist()
controller.posLeft.value = np.eye(4).tolist()

controller.vRight.recompute(0)
controller.vLeft.recompute(0)
controller.gainRight.recompute(0)
controller.gainLeft.recompute(0)

# There is more pressure on the right foot.
# Therefore, the right foot must go up to reduce it
vRight = [0.] * 2 + [100.] + [0.] * 3
vLeft = [0.] * 2 + [-100.] + [0.] * 3

print("Expected vRight: %s" % str(vRight))
print("Actual vRight:   %s" % str(controller.vRight.value))
print("Expected vLeft:  %s" % str(vLeft))
print("Actual vLeft:    %s" % str(controller.vLeft.value))
print()

assertApprox(vRight, controller.vRight.value)
assertApprox(vLeft, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assertApprox(gainDouble, controller.gainRight.value)
assertApprox(gainDouble, controller.gainLeft.value)

print("---- Left support ----")
controller.phase.value = 1

controller.vRight.recompute(1)
controller.vLeft.recompute(1)
controller.gainRight.recompute(1)
controller.gainLeft.recompute(1)
print("vRight:   %s" % str(controller.vRight.value))
print("vLeft:    %s" % str(controller.vLeft.value))
print()

assertApprox([0.] * 6, controller.vRight.value)
assertApprox([0.] * 6, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assertApprox(gainSwing, controller.gainRight.value)
assertApprox(gainStance, controller.gainLeft.value)

print("---- Right support ----")
controller.phase.value = -1

controller.vRight.recompute(2)
controller.vLeft.recompute(2)
controller.gainRight.recompute(2)
controller.gainLeft.recompute(2)
print("vRight:   %s" % str(controller.vRight.value))
print("vLeft:    %s" % str(controller.vLeft.value))
print()

assertApprox([0.] * 6, controller.vRight.value)
assertApprox([0.] * 6, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assertApprox(gainStance, controller.gainRight.value)
assertApprox(gainSwing, controller.gainLeft.value)

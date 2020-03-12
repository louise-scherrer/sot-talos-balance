from __future__ import print_function

from sot_talos_balance.saturation import Saturation

saturation = Saturation("sat")

print("Commands:")
print(saturation.commands())

print("\nSignals (at creation):")
saturation.displaySignals()

x = [1.0]
y = [1.0]
k = 1.0
xLim = [2.0]
yLim = [1.0]

saturation.x.value = x
saturation.y.value = y
saturation.k.value = k
saturation.xLim.value = xLim
saturation.yLim.value = yLim

saturation.yOut.recompute(0)
print("\nyOut: %s - Expected : 1.0" % (saturation.yOut.value, ))
assert saturation.yOut.value[0] == 1.0

yLim = [0.5]
saturation.yLim.value = yLim
saturation.yOut.recompute(1)
print("\nyOut: %s - Expected : 0.5" % (saturation.yOut.value, ))
assert saturation.yOut.value[0] == 0.5

yLim = [1.0]
x = [1.5]
saturation.yLim.value = yLim
saturation.x.value = x
saturation.yOut.recompute(2)
print("\nyOut: %s - Expected : 0.5" % (saturation.yOut.value, ))
assert saturation.yOut.value[0] == 0.5

x = [2.5]
saturation.x.value = x
saturation.yOut.recompute(3)
print("\nyOut: %s - Expected : 0.0" % (saturation.yOut.value, ))
assert saturation.yOut.value[0] == 0.0

x = [-1.5]
saturation.x.value = x
saturation.yOut.recompute(4)
print("\nyOut: %s - Expected : 0.5" % (saturation.yOut.value, ))
assert saturation.yOut.value[0] == 0.5

from __future__ import print_function

from sot_talos_balance.com_admittance_controller import ComAdmittanceController

controller = ComAdmittanceController("ciao")

print("Commands:")
print(controller.commands())

print("\nSignals (at creation):")
controller.displaySignals()

Kp = [10.0, 10.0, 0.0]
ddcomDes = tuple(3 * [0.0])
zmpDes = tuple(3 * [0.0])
zmp = tuple(3 * [0.0])

controller.Kp.value = Kp
controller.ddcomDes.value = ddcomDes
controller.zmpDes.value = zmpDes
controller.zmp.value = zmp

print()
print("Kp:       %s" % (controller.Kp.value, ))
print("ddcomDes: %s" % (controller.ddcomDes.value, ))
print("zmpDes:   %s" % (controller.zmpDes.value, ))
print("zmp:      %s" % (controller.zmp.value, ))

com = tuple(3 * [0.0])
dcom = tuple(3 * [0.0])
dt = 1

controller.init(dt)

controller.setState(com, dcom)
controller.comRef.recompute(0)
controller.dcomRef.recompute(0)

print()
print("comRef:  %s" % (controller.comRef.value, ))
assert controller.comRef.value == com
print("dcomRef: %s" % (controller.dcomRef.value, ))
assert controller.dcomRef.value == dcom

ddcomDes = tuple(3 * [1.0])
controller.ddcomDes.value = ddcomDes

print()
print("ddcomDes: %s" % (controller.ddcomDes.value, ))

controller.dcomRef.recompute(1)
controller.comRef.recompute(1)

print()

print("ddcomRef: %s" % (controller.ddcomRef.value, ))
ddcomRef = tuple([ddcomDes[i] + Kp[i] * (zmp[i] - zmpDes[i]) for i in range(3)])
assert controller.ddcomRef.value == ddcomRef

print("dcomRef:  %s" % (controller.dcomRef.value, ))
dcomRef = tuple([dcom[i] + ddcomRef[i] * dt for i in range(3)])
assert controller.dcomRef.value == dcomRef

print("comRef:   %s" % (controller.comRef.value, ))
comRef = tuple([com[i] + dcom[i] * dt + 0.5 * ddcomRef[i] * dt * dt for i in range(3)])
assert controller.comRef.value == comRef

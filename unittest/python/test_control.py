from __future__ import print_function

from sot_talos_balance.joint_position_controller import JointPositionController

controller = JointPositionController("ciao")

print("Commands:")
print(controller.commands())

print("\nSignals (at creation):")
controller.displaySignals()

N_JOINTS = 2

controller.Kp.value = tuple(N_JOINTS * [10.0])
controller.state.value = tuple([0.0] * 6 + N_JOINTS * [0.0])
controller.qDes.value = tuple(N_JOINTS * [1.0])
controller.dqDes.value = tuple(N_JOINTS * [0.0])

controller.init(N_JOINTS)

controller.dqRef.recompute(1)

print("\nKp: %s" % (controller.Kp.value, ))
print("\nq: %s" % (controller.state.value, ))
print("qDes: %s" % (controller.qDes.value, ))
print("dqDes: %s" % (controller.dqDes.value, ))

print("\ndqRef: %s" % (controller.dqRef.value, ))

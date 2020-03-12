from __future__ import print_function

import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import Example, create_parameter_server

# --- Parameter server ---
print("--- Parameter server ---")

dt = 1e-3
robot_name = 'robot'
param_server = create_parameter_server(param_server_conf, dt)

# --- Example ---
print("--- Example ---")

ex = Example("ciao")

print("Commands:")
print(ex.commands())

print("\nSignals (at creation):")
ex.displaySignals()

ex.firstAddend.value = 0.1
ex.secondAddend.value = 0.3

print("\nSignals (after pugging):")
ex.displaySignals()

ex.init(robot_name)

ex.sum.recompute(1)

print("\nInputs: %f, %f" % (ex.firstAddend.value, ex.secondAddend.value))
print("Output: %f" % ex.sum.value)

ex.nbJoints.recompute(1)
print("nbJoints: %d" % ex.nbJoints.value)

import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import Bunch, Example, TalosBaseEstimator, create_parameter_server

dt = 0.001
conf = Bunch()
robot_name = 'robot'

param_server = create_parameter_server(param_server_conf, dt)

base_estimator = TalosBaseEstimator('base_estimator')
base_estimator.init(dt, robot_name)

example = Example('example')
example.firstAddend.value = 0.
example.secondAddend.value = 0.
example.init(robot_name)

from __future__ import print_function

import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox
from rospkg import RosPack

import sot_talos_balance.talos.parameter_server_conf as parameter_server_conf
from sot_talos_balance.create_entities_utils import Bunch, DcmEstimator, ParameterServer

dt = 0.001
conf = Bunch()
robot_name = 'robot'

rospack = RosPack()
urdfPath = rospack.get_path('talos_data') + "/urdf/talos_reduced.urdf"
urdfDir = [rospack.get_path('talos_data') + "/../"]

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
model.lowerPositionLimit = np.vstack((np.matrix([-1.] * 7).T, model.lowerPositionLimit[7:]))
model.upperPositionLimit = np.vstack((np.matrix([-1.] * 7).T, model.upperPositionLimit[7:]))
data = model.createData()
q = pin.randomConfiguration(model)
v = pin.utils.rand(model.nv)

pin.centerOfMass(model, data, q, v)
print("Expected:")
print("CoM position value: {0}".format(tuple(data.com[0].flat)))
print("CoM velocity value: {0}".format(tuple(data.vcom[0].flat)))

conf.param_server = parameter_server_conf
param_server = ParameterServer("param_server")
param_server.init(dt, conf.param_server.urdfFileName, robot_name)
param_server.setJointsUrdfToSot(conf.param_server.urdftosot)
param_server.setRightFootForceSensorXYZ(conf.param_server.rightFootSensorXYZ)
param_server.setRightFootSoleXYZ(conf.param_server.rightFootSoleXYZ)

dcm_estimator = DcmEstimator('dcm_estimator')
dcm_estimator.q.value = list(q.flat)
dcm_estimator.v.value = list(v.flat)
dcm_estimator.init(dt, robot_name)
dcm_estimator.c.recompute(1)
dcm_estimator.dc.recompute(1)
print("Computed:")
print("CoM position value: {0}".format(dcm_estimator.c.value))
assertApprox(data.com[0], np.matrix(dcm_estimator.c.value).T, 3)
print("CoM velocity value: {0}".format(dcm_estimator.dc.value))
assertApprox(data.vcom[0], np.matrix(dcm_estimator.dc.value).T, 3)

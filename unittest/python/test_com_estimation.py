import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox
from rospkg import RosPack

import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.create_entities_utils import (DcmEstimator, EulerToQuat, TalosBaseEstimator,
                                                     create_parameter_server, plug)

# --- General ---
print("--- General ---")

dt = 0.001
robot_name = 'robot'

halfSitting = [
    0.0,
    0.0,
    1.018213,
    0.00,
    0.0,
    0.0,
    1.0,  # Free flyer
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # Left Leg
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # Right Leg
    0.0,
    0.006761,  # Chest
    0.25847,
    0.173046,
    -0.0002,
    -0.525366,
    0.0,
    -0.0,
    0.1,
    -0.005,  # Left Arm
    -0.25847,
    -0.173046,
    0.0002,
    -0.525366,
    0.0,
    0.0,
    0.1,
    -0.005,  # Right Arm
    0.,
    0.  # Head
]

q = np.matrix(halfSitting).T
print("q:")
print(q.flatten().tolist()[0])

rospack = RosPack()
urdfPath = rospack.get_path('talos_data') + "/urdf/talos_reduced.urdf"
urdfDir = [rospack.get_path('talos_data') + "/../"]

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()
com = pin.centerOfMass(model, data, q)
pin.updateFramePlacements(model, data)
m = data.mass[0]

print("com:")
print(com.flatten().tolist()[0])

leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos = data.oMf[leftId]
print("%s: %d" % (leftName, leftId))
print(leftPos)

rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
print("%s: %d" % (rightName, rightId))
print(rightPos)

g = 9.81
fz = m * g / 2.0
forceLeft = [0.0, 0.0, fz]
forceRight = [0.0, 0.0, fz]
lever = float(com[0] - rightPos.translation[0])
tauy = -fz * lever
wrenchLeft = forceLeft + [0.0, tauy, 0.0]
wrenchRight = forceRight + [0.0, tauy, 0.0]

centerTranslation = (data.oMf[rightId].translation + data.oMf[leftId].translation) / 2 + np.matrix(
    param_server_conf.rightFootSoleXYZ).T

centerPos = pin.SE3(rightPos.rotation, centerTranslation)
print("Center of feet:")
print(centerPos)

comRel = centerPos.actInv(com)
print("Relative CoM:")
print(comRel.flatten().tolist()[0])

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf, dt)

# --- Base estimator ---
print("--- Base estimator ---")

conf = base_estimator_conf
base_estimator = TalosBaseEstimator('base_estimator')
base_estimator.init(dt, robot_name)

base_estimator.joint_positions.value = halfSitting[7:]
base_estimator.forceLLEG.value = wrenchLeft
base_estimator.forceRLEG.value = wrenchRight
base_estimator.dforceLLEG.value = [0.0] * 6
base_estimator.dforceRLEG.value = [0.0] * 6
base_estimator.joint_velocities.value = [0.0] * (model.nv - 6)
base_estimator.imu_quaternion.value = [0.0] * 3 + [1.0]
base_estimator.gyroscope.value = [0.0] * 3
base_estimator.accelerometer.value = [0.0] * 3

base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses
base_estimator.w_lf_in.value = conf.w_lf_in
base_estimator.w_rf_in.value = conf.w_rf_in
# base_estimator.set_imu_weight(conf.w_imu) # TEMP!
base_estimator.set_imu_weight(0.)
base_estimator.set_stiffness_right_foot(conf.K)
base_estimator.set_stiffness_left_foot(conf.K)
base_estimator.set_zmp_std_dev_right_foot(conf.std_dev_zmp)
base_estimator.set_zmp_std_dev_left_foot(conf.std_dev_zmp)
base_estimator.set_normal_force_std_dev_right_foot(conf.std_dev_fz)
base_estimator.set_normal_force_std_dev_left_foot(conf.std_dev_fz)
base_estimator.set_zmp_margin_right_foot(conf.zmp_margin)
base_estimator.set_zmp_margin_left_foot(conf.zmp_margin)
base_estimator.set_normal_force_margin_right_foot(conf.normal_force_margin)
base_estimator.set_normal_force_margin_left_foot(conf.normal_force_margin)
base_estimator.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
base_estimator.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)

base_estimator.q.recompute(0)
print(base_estimator.q.value)
print(len(base_estimator.q.value))
base_estimator.v.recompute(0)
print(base_estimator.v.value)
print(len(base_estimator.v.value))

# --- Conversion ---
print("--- Conversion ---")

e2q = EulerToQuat('e2q')
plug(base_estimator.q, e2q.euler)
e2q.quaternion.recompute(0)
print(e2q.quaternion.value)
print(len(e2q.quaternion.value))
q_est = np.matrix(e2q.quaternion.value).T

# --- Raw q difference ---
print("--- Raw q difference ---")
q_diff = q_est - q
print(q_diff.flatten().tolist()[0])

# --- Estimated feet ---
print("--- Estimated feet ---")
data2 = model.createData()
pin.framesForwardKinematics(model, data2, q_est)
print(data2.oMf[rightId])
print(data2.oMf[leftId])

# --- DCM estimator ---
print("--- DCM estimator ---")

dcm_estimator = DcmEstimator('dcm_estimator')
dcm_estimator.init(dt, robot_name)
plug(e2q.quaternion, dcm_estimator.q)
plug(base_estimator.v, dcm_estimator.v)
dcm_estimator.c.recompute(0)
print(dcm_estimator.c.value)

# --- Direct CoM ---
print("--- Direct CoM ---")
print(com.flatten().tolist()[0])

# --- Raw CoM difference ---
print("--- Raw CoM difference ---")
com_rawdiff = np.matrix(dcm_estimator.c.value).T - com
print(com_rawdiff.flatten().tolist()[0])

# --- Relative CoM difference ---
print("--- Relative CoM difference ---")
com_reldiff = np.matrix(dcm_estimator.c.value).T - comRel
print(com_reldiff.flatten().tolist()[0])

assertApprox(np.matrix(dcm_estimator.c.value).T, comRel, 3)

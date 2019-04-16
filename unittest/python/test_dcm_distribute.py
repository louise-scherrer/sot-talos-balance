from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import numpy as np
from math import sqrt
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox

# --- General ---
print("--- General ---")

dt = 0.001
robot_name = 'robot'

halfSitting = [0.0, 0.0,  1.018213,  0.00  ,  0.0, 0.0, 1.0,                         #Free flyer
               0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, #Left Leg
               0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, #Right Leg
               0.0 ,  0.006761,                                                 #Chest
               0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,        #Left Arm
              -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,      #Right Arm
               0.,  0.                                                          #Head
]

q = np.matrix(halfSitting).T
print("q:")
print(q.flatten().tolist()[0])

from rospkg import RosPack
rospack = RosPack()
urdfPath = rospack.get_path('talos_data')+"/urdf/talos_reduced.urdf"
urdfDir = [rospack.get_path('talos_data')+"/../"]

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()
com = pin.centerOfMass(model,data,q)
pin.updateFramePlacements(model,data)
m = data.mass[0]
h = float(com[2])
g = 9.81
omega = sqrt(g/h)

leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos  = data.oMf[leftId]

rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]

centerTranslation = ( data.oMf[rightId].translation + data.oMf[leftId].translation )/2 + np.matrix(param_server_conf.rightFootSoleXYZ).T
centerPos = pin.SE3(rightPos.rotation,centerTranslation)
comRel = centerPos.actInv(com)

fz = m*g
force      = [0.0, 0.0, fz]
forceLeft  = [0.0, 0.0, fz/2]
forceRight = [0.0, 0.0, fz/2]
tau        = np.cross(comRel,np.matrix(force).T,axis=0)
wrench      = force      + tau.flatten().tolist()
wrenchLeft  = forceLeft  + (tau/2).flatten().tolist()
wrenchRight = forceRight + (tau/2).flatten().tolist()

print( "desired wrench: %s" % str(wrench) )
print( "expected left wrench: %s"  % str(wrenchLeft) )
print( "expected right wrench: %s" % str(wrenchRight) )

# --- Desired CoM, DCM and ZMP
comDes = tuple(comRel.flatten().tolist()[0])
dcmDes = comDes
zmpDes = comDes[:2] + (0.0,)

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf,dt)

# --- DCM controller
Kp_dcm = [0.0,0.0,0.0]
Ki_dcm = [0.0,0.0,0.0]
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = m
dcm_controller.omega.value = omega

dcm_controller.com.value = comDes
dcm_controller.dcm.value = comDes

dcm_controller.zmpDes.value = zmpDes
dcm_controller.dcmDes.value = dcmDes

dcm_controller.init(dt)

# --- Wrench distribution ---
print("--- Wrench distribution ---")

distribute = DistributeWrench('distribute')

distribute.q.value = halfSitting
plug(dcm_controller.wrenchRef,distribute.wrenchDes)

distribute.init(robot_name)

distribute.zmpRef.recompute(0)

print( "reference wrench: %s" % str(dcm_controller.wrenchRef.value) )
assertApprox(wrench,dcm_controller.wrenchRef.value,3)
print( "resulting wrench: %s" % str(distribute.wrenchRef.value) )
assertApprox(wrench,distribute.wrenchRef.value,3)
print( "resulting left wrench: %s"  % str(distribute.wrenchLeft.value) )
assertApprox(wrenchLeft,distribute.wrenchLeft.value,3)
print( "resulting right wrench: %s" % str(distribute.wrenchRight.value) )
assertApprox(wrenchRight,distribute.wrenchRight.value,3)


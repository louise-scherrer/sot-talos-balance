# flake8: noqa
import sot_talos_balance.talos.hip_flexibility_compensation_conf as hipFlexCompConfig
import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- EXPERIMENTAL SET UP ------------------------------------------------------

device = 'simu'
# WARNING : The device control type is set to NO INTEGRATION
# The control HAS TO BE in POSITION
robot.device.setControlInputType("noInteg")
N_JOINTS = 32
robot_name = "robot"

# --- SET INITIAL CONFIGURATION ------------------------------------------------

robot.halfSitting = [0., 0., 1.018213, 0., 0., 0.]  # Base
robot.halfSitting += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Left Leg
robot.halfSitting += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Right Leg
robot.halfSitting += [0.0, 0.006761]  # Chest
robot.halfSitting += [0.25847, 0.173046, -0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Left Arm
robot.halfSitting += [-0.25847, -0.173046, 0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Right Arm
robot.halfSitting += [0., 0.]  # Head
robot.device.set(robot.halfSitting)

# --- CREATE ENTITIES ----------------------------------------------------------

robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)

# --- HIP TASK ----------------------------------------------------------------

robot.hipComp = create_hip_flexibility_compensation(robot, hipFlexCompConfig, robot_name)
robot.hipComp.q_des.value = robot.halfSitting

# --- PLUG CONTROL ----------------------------------------------------------------
# WARNING : Set the control to halfSitting position !
# Plug the hip_flexibility_compensation in the test file
robot.device.control.value = robot.halfSitting

# # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.hipComp, 'delta_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_cmd', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'tau', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'tau_filt', robot=robot, data_type='vector')

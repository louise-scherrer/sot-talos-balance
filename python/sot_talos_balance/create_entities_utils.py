from sot_talos_balance.talos_control_manager import TalosControlManager
from sot_talos_balance.example import Example
from dynamic_graph.sot.core.parameter_server import ParameterServer
from dynamic_graph.tracer_real_time import TracerRealTime
from time import sleep
from sot_talos_balance.talos_base_estimator import TalosBaseEstimator
from dynamic_graph.sot.core.madgwickahrs import MadgwickAHRS
from sot_talos_balance.dcm_estimator import DcmEstimator
from sot_talos_balance.ft_calibration import FtCalibration
from sot_talos_balance.ft_wrist_calibration import FtWristCalibration

from sot_talos_balance.euler_to_quat import EulerToQuat
from sot_talos_balance.quat_to_euler import QuatToEuler
from sot_talos_balance.pose_quaternion_to_matrix_homo import PoseQuaternionToMatrixHomo

from dynamic_graph.sot.core.operator import Mix_of_vector
from dynamic_graph.sot.core.operator import Selec_of_vector
from dynamic_graph.sot.core.operator import Component_of_vector
from dynamic_graph.sot.core.operator import MatrixHomoToPoseQuaternion
from dynamic_graph.sot.core.operator import PoseRollPitchYawToMatrixHomo
from dynamic_graph.sot.core.operator import MatrixHomoToPoseRollPitchYaw
from sot_talos_balance.boolean_identity import BooleanIdentity
from sot_talos_balance.int_identity import IntIdentity
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.simple_pid import SimplePID
from sot_talos_balance.simple_pidd import SimplePIDD
from sot_talos_balance.joint_position_controller import JointPositionController
from sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController
from sot_talos_balance.admittance_controller_end_effector import AdmittanceControllerEndEffector
from sot_talos_balance.ankle_admittance_controller import AnkleAdmittanceController
from sot_talos_balance.foot_force_difference_controller import FootForceDifferenceController
from sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from sot_talos_balance.com_admittance_controller import ComAdmittanceController
from sot_talos_balance.dcm_controller import DcmController
from sot_talos_balance.dcm_com_controller import DcmComController
from sot_talos_balance.simple_zmp_estimator import SimpleZmpEstimator
from sot_talos_balance.simple_distribute_wrench import SimpleDistributeWrench
from sot_talos_balance.distribute_wrench import DistributeWrench
from sot_talos_balance.simple_reference_frame import SimpleReferenceFrame
from sot_talos_balance.state_transformation import StateTransformation
from sot_talos_balance.dummy_walking_pattern_generator import DummyWalkingPatternGenerator
from sot_talos_balance.ankle_joint_selector import AnkleJointSelector
from sot_talos_balance.qualisys_client import QualisysClient
from sot_talos_balance.hip_flexibility_compensation import HipFlexibilityCompensation
from sot_talos_balance.simple_state_integrator import SimpleStateIntegrator
from sot_talos_balance.delay import DelayVector
from sot_talos_balance.round_double_to_int import RoundDoubleToInt

# python
from sot_talos_balance.utils import filter_utils
from sot_talos_balance.utils.sot_utils import Bunch

from dynamic_graph import plug

import numpy as np

N_JOINTS = 32

def create_qualisys_client(address):
    mocap = QualisysClient('mocap')
    mocap.setMocapIPAdress(address)
    mocap.init()
    return mocap


# helper function. May need to move somewhere else
def rotation_matrix_to_rpy(R):
    rx = np.arctan2(R[2, 1], R[2, 2])
    ry = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2]))
    rz = np.arctan2(R[1, 0], R[0, 0])

    return (rx, ry, rz)


def create_extend_mix(n_in, n_out):
    assert n_out > n_in
    mix_of_vector = Mix_of_vector("mix " + str(n_in) + "-" + str(n_out))

    mix_of_vector.setSignalNumber(3)

    n_diff = n_out - n_in
    mix_of_vector.addSelec(1, 0, n_diff)
    mix_of_vector.addSelec(2, n_diff, n_in)

    mix_of_vector.default.value = [0.0] * n_out
    mix_of_vector.signal("sin1").value = [0.0] * n_diff
    mix_of_vector.signal("sin2").value = [2.0] * n_in

    return mix_of_vector


def create_scalar_trajectory_generator(dt, init_value, name):
    trajGen = NdTrajectoryGenerator(name)
    trajGen.initial_value.value = [init_value]
    trajGen.trigger.value = 1.0
    trajGen.init(dt, 1)
    return trajGen


def create_joint_trajectory_generator(dt, robot):
    jtg = NdTrajectoryGenerator("jtg")
    jtg.initial_value.value = robot.device.state.value[6:]
    jtg.trigger.value = 1.0
    jtg.init(dt, N_JOINTS)
    return jtg


def create_config_trajectory_generator(dt, robot):
    N_CONFIG = N_JOINTS + 6
    jtg = NdTrajectoryGenerator("jtg")
    jtg.initial_value.value = robot.device.state.value
    jtg.trigger.value = 1.0
    jtg.init(dt, N_CONFIG)
    return jtg

def create_torque_trajectory_generator(dt, robot):
    N_CONFIG = N_JOINTS + 6
    jtg = NdTrajectoryGenerator("torqueTrajGen")
    jtg.initial_value.value = [0.] * N_CONFIG
    jtg.trigger.value = 1.0
    jtg.init(dt, N_CONFIG)
    return jtg

def create_com_trajectory_generator(dt, robot):
    comTrajGen = NdTrajectoryGenerator("comTrajGen")
    comTrajGen.initial_value.value = robot.dynamic.com.value
    comTrajGen.trigger.value = 1.0
    comTrajGen.init(dt, 3)
    return comTrajGen


def create_zmp_trajectory_generator(dt, robot):
    comTrajGen = NdTrajectoryGenerator("zmpTrajGen")
    zmp = list(robot.dynamic.com.value)
    zmp[2] = 0.0
    comTrajGen.initial_value.value = zmp
    comTrajGen.trigger.value = 1.0
    comTrajGen.init(dt, 3)
    return comTrajGen


def create_position_trajectory_generator(dt, robot, signal_name):
    trajGen = NdTrajectoryGenerator(signal_name + "PosTrajGen")

    M = robot.dynamic.signal(signal_name).value
    v = [M[i][3] for i in range(3)]
    trajGen.initial_value.value = v

    trajGen.trigger.value = 1.0
    trajGen.init(dt, 3)
    return trajGen


def create_orientation_rpy_trajectory_generator(dt, robot, signal_name):
    trajGen = NdTrajectoryGenerator(signal_name + "OrientationTrajGen")

    M = robot.dynamic.signal(signal_name).value
    v = list(rotation_matrix_to_rpy(np.array(M)[:3, :3]))
    trajGen.initial_value.value = v

    trajGen.trigger.value = 1.0
    trajGen.init(dt, 3)
    return trajGen


def create_pose_rpy_trajectory_generator(dt, robot, signal_name):
    trajGen = NdTrajectoryGenerator(signal_name + "TrajGen")

    M = robot.dynamic.signal(signal_name).value
    pos = [M[i][3] for i in range(3)]
    euler = list(rotation_matrix_to_rpy(np.array(M)[:3, :3]))
    v = pos + euler
    trajGen.initial_value.value = v

    trajGen.trigger.value = 1.0
    trajGen.init(dt, 6)
    return trajGen


def create_joint_controller(Kp):
    controller = JointPositionController("posctrl")
    controller.Kp.value = Kp
    return controller


def create_end_effector_admittance_controller(robot, endEffector, name):
    timeStep = robot.timeStep
    controller = AdmittanceControllerEndEffector(name)

    # Filter and plug the force from force calibrator
    if endEffector == 'rightWrist':
        plug(robot.forceCalibrator.rightWristForceOut, controller.force)
    elif endEffector == 'leftWrist':
        plug(robot.forceCalibrator.leftWristForceOut, controller.force)
    else:
        print('Error in create_end_effector_admittance_controller : end \
        effector unknown')

    plug(robot.e2q.quaternion, controller.q)

    controller.Kp.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller.Kd.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller.w_forceDes.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller.dqSaturation.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if endEffector == 'rightWrist':
        controller.init(timeStep, "wrist_right_ft_link", 'arm_right_7_joint')
    elif endEffector == 'leftWrist':
        controller.init(timeStep, "wrist_left_ft_link", 'arm_left_7_joint')
    else:
        print('Error in create_end_effector_admittance_controller : end \
        effector unknown')

    return controller


def create_joint_admittance_controller(joint, Kp, dt, robot, filter=False):
    controller = SimpleAdmittanceController("jadmctrl")
    controller.Kp.value = Kp

    robot.stateselec = Selec_of_vector("state_selec")
    robot.stateselec.selec(joint + 6, joint + 7)
    plug(robot.device.state, robot.stateselec.sin)
    plug(robot.stateselec.sout, controller.state)

    robot.tauselec = Selec_of_vector("tau_selec")
    robot.tauselec.selec(joint, joint + 1)
    if filter and hasattr(robot, 'device_filters'):
        plug(robot.device_filters.torque_filter.x_filtered, robot.tauselec.sin)
    else:
        plug(robot.device.ptorque, robot.tauselec.sin)
    plug(robot.tauselec.sout, controller.tau)

    controller.tauDes.value = [0.0]
    controller.init(dt, 1)
    controller.setPosition([robot.device.state.value[joint + 6]])
    return controller


def create_hip_flexibility_compensation(robot, conf, robot_name='robot'):
    timeStep = robot.timeStep
    hipComp = HipFlexibilityCompensation("hipFlexCompensation")
    hipComp.K_l.value = conf.flexibility_left
    hipComp.K_r.value = conf.flexibility_right
    hipComp.q_des.value = robot.dynamic.getDimension() * [0.]
    plug(robot.device.ptorque, hipComp.tau)
    hipComp.init(timeStep, robot_name)

    hipComp.setAngularSaturation(conf.angular_saturation)
    hipComp.setRateLimiter(conf.rate_limiter)
    hipComp.setTorqueLowPassFilterFrequency(conf.torque_low_pass_freq)
    return hipComp


def create_ankle_admittance_controller(gains, robot, side, name):
    controller = AnkleAdmittanceController(name)

    # Filter and plug the force from force calibrator
    if side == "right":
        plug(robot.ftc.right_foot_force_out, controller.wrench)
    elif side == "left":
        plug(robot.ftc.left_foot_force_out, controller.wrench)
    else:
        print('Error in create_ankle_admittance_controller : side unknown')

    controller.gainsXY.value = gains
    if side == "right":
        plug(robot.wrenchDistributor.copRight, controller.pRef)
    elif side == "left":
        plug(robot.wrenchDistributor.copLeft, controller.pRef)
    else:
        print('Error in create_ankle_admittance_controller : side unknown')

    controller.init()

    return controller


def create_device_filters(robot, dt):
    robot.pselec = Selec_of_vector("pselec")
    robot.pselec.selec(6, 6 + N_JOINTS)
    plug(robot.device.state, robot.pselec.sin)

    robot.vselec = Selec_of_vector("vselec")
    robot.vselec.selec(6, 6 + N_JOINTS)
    plug(robot.device.velocity, robot.vselec.sin)

    filters = Bunch()
    filters.joints_kin = filter_utils.create_chebi1_checby2_series_filter("joints_kin", dt, N_JOINTS)
    filters.ft_RF_filter = filter_utils.create_butter_lp_filter_Wn_04_N_2("ft_RF_filter", dt, 6)
    filters.ft_LF_filter = filter_utils.create_butter_lp_filter_Wn_04_N_2("ft_LF_filter", dt, 6)
    filters.ft_RH_filter = filter_utils.create_butter_lp_filter_Wn_04_N_2("ft_RH_filter", dt, 6)
    filters.ft_LH_filter = filter_utils.create_butter_lp_filter_Wn_04_N_2("ft_LH_filter", dt, 6)
    filters.torque_filter = filter_utils.create_chebi1_checby2_series_filter("ptorque_filter", dt, N_JOINTS)
    filters.acc_filter = filter_utils.create_chebi1_checby2_series_filter("acc_filter", dt, 3)
    filters.gyro_filter = filter_utils.create_chebi1_checby2_series_filter("gyro_filter", dt, 3)
    filters.vel_filter = filter_utils.create_butter_lp_filter_Wn_04_N_2("vel_filter", dt, N_JOINTS)

    #    plug(robot.pselec.sout,                               filters.joints_kin.x)
    plug(robot.device.joint_angles, filters.joints_kin.x)
    plug(robot.device.forceRLEG, filters.ft_RF_filter.x)
    plug(robot.device.forceLLEG, filters.ft_LF_filter.x)
    plug(robot.device.forceRARM, filters.ft_RH_filter.x)
    plug(robot.device.forceLARM, filters.ft_LH_filter.x)
    plug(robot.device.ptorque, filters.torque_filter.x)
    plug(robot.vselec.sout, filters.vel_filter.x)

    plug(robot.device.accelerometer, filters.acc_filter.x)
    plug(robot.device.gyrometer, filters.gyro_filter.x)

    return filters


def create_be_filters(robot, dt):
    be_filters = Bunch()
    be_filters.test = filter_utils.create_chebi1_checby2_series_filter("test_filter", dt, N_JOINTS)
    plug(robot.base_estimator.q, be_filters.test.x)
    return be_filters


def create_ctrl_manager(conf, dt, robot_name='robot'):
    ctrl_manager = TalosControlManager("ctrl_man")
    ctrl_manager.init(dt, robot_name)
    ctrl_manager.u_max.value = conf.NJ * (conf.CTRL_MAX, )
    # Init should be called before addCtrlMode
    # because the size of state vector must be known.
    return ctrl_manager


def create_base_estimator(robot, dt, conf, robot_name="robot"):
    base_estimator = TalosBaseEstimator('base_estimator')
    base_estimator.init(dt, robot_name)
    # device.state, device.joint_angles or device.motor_angles ?
    # plug(robot.pselec.sout, base_estimator.joint_positions)
    plug(robot.device.joint_angles, base_estimator.joint_positions)
    plug(robot.device_filters.ft_LF_filter.x_filtered, base_estimator.forceLLEG)
    plug(robot.device_filters.ft_RF_filter.x_filtered, base_estimator.forceRLEG)
    plug(robot.device_filters.ft_LF_filter.dx, base_estimator.dforceLLEG)
    plug(robot.device_filters.ft_RF_filter.dx, base_estimator.dforceRLEG)

    plug(robot.vselec.sout, base_estimator.joint_velocities)
    # plug(robot.device_filters.vel_filter.x_filtered,     base_estimator.joint_velocities)
    plug(robot.imu_filters.imu_quat, base_estimator.imu_quaternion)
    plug(robot.device_filters.gyro_filter.x_filtered, base_estimator.gyroscope)
    plug(robot.device_filters.acc_filter.x_filtered, base_estimator.accelerometer)
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses
    # base_estimator.w_lf_in.value = conf.w_lf_in
    # base_estimator.w_rf_in.value = conf.w_rf_in
    base_estimator.set_imu_weight(conf.w_imu)
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

    return base_estimator


def create_imu_filters(robot, dt):
    imu_filter = MadgwickAHRS('imu_filter')
    imu_filter.init(dt)
    imu_filter.set_imu_quat([0., 1., 0., 0.])  # [w, x, y, z]
    imu_filter.setBeta(1e-3)
    plug(robot.device_filters.acc_filter.x_filtered, imu_filter.accelerometer)  # no IMU compensation
    plug(robot.device_filters.gyro_filter.x_filtered, imu_filter.gyroscope)  # no IMU compensation
    return imu_filter


def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName)
    filename = '{0}-{1}'.format(entity.name, signalName)
    tracer.add(signal, filename)


def addSignalsToTracer(tracer, device, outputs):
    for sign in outputs:
        addTrace(tracer, device, sign)
    return


def create_tracer(robot, entity, tracer_name, outputs=None):
    tracer = TracerRealTime(tracer_name)
    tracer.setBufferSize(80 * (2**20))
    tracer.open('/tmp', 'dg_', '.dat')
    robot.device.after.addSignal('{0}.triger'.format(tracer.name))
    if outputs is not None:
        addSignalsToTracer(tracer, entity, outputs)
    return tracer


def reset_tracer(device, tracer):
    tracer.stop()
    sleep(0.2)
    tracer.close()
    sleep(0.2)
    tracer.clear()
    sleep(0.2)
    tracer = create_tracer(device, tracer.name)
    return tracer


def dump_tracer(tracer):
    tracer.stop()
    sleep(0.2)
    tracer.dump()
    sleep(0.2)
    tracer.close()


def create_rospublish(robot, name):
    from dynamic_graph.ros import RosPublish
    rospub = RosPublish(name)
    robot.device.after.addSignal(rospub.name + '.trigger')
    return rospub


def create_topic(rospub, entity, signalName, robot=None, data_type='vector'):
    # check needed to prevent creation of broken topic
    if not entity.hasSignal(signalName):
        raise AttributeError('Entity %s does not have signal %s' % (entity.name, signalName))
    rospub_signalName = '{0}_{1}'.format(entity.name, signalName)
    topicname = '/sot/{0}/{1}'.format(entity.name, signalName)
    rospub.add(data_type, rospub_signalName, topicname)
    plug(entity.signal(signalName), rospub.signal(rospub_signalName))
    if robot is not None:
        robot.device.after.addSignal('{0}.{1}'.format(entity.name, signalName))


def create_dummy_dcm_estimator(robot):
    from math import sqrt
    estimator = DummyDcmEstimator("dummy")
    robot.dynamic.com.recompute(0)
    mass = robot.dynamic.data.mass[0]
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g / h)

    estimator.mass.value = mass
    estimator.omega.value = omega
    plug(robot.dynamic.com, estimator.com)
    plug(robot.dynamic.momenta, estimator.momenta)
    estimator.init()
    return estimator


def create_cdc_dcm_estimator(robot):
    from math import sqrt
    estimator = DummyDcmEstimator("dummy")
    robot.dynamic.com.recompute(0)
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g / h)

    estimator.mass.value = 1.0
    estimator.omega.value = omega
    plug(robot.cdc_estimator.c, estimator.com)
    plug(robot.cdc_estimator.dc, estimator.momenta)
    estimator.init()
    return estimator


def create_com_admittance_controller(Kp, dt, robot):
    controller = ComAdmittanceController("comAdmCtrl")
    controller.Kp.value = Kp
    plug(robot.dynamic.zmp, controller.zmp)
    robot.dynamic.zmp.recompute(0)
    controller.zmpDes.value = robot.dynamic.zmp.value
    controller.ddcomDes.value = [0.0, 0.0, 0.0]

    controller.init(dt)
    robot.dynamic.com.recompute(0)
    controller.setState(robot.dynamic.com.value, [0.0, 0.0, 0.0])
    return controller


def create_dcm_controller(Kp, Ki, dt, robot, dcmSignal):
    from math import sqrt
    controller = DcmController("dcmCtrl")
    robot.dynamic.com.recompute(0)
    mass = robot.dynamic.data.mass[0]
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g / h)

    controller.Kp.value = Kp
    controller.Ki.value = Ki
    controller.decayFactor.value = 0.2
    controller.mass.value = mass
    controller.omega.value = omega

    plug(robot.dynamic.com, controller.com)
    plug(dcmSignal, controller.dcm)

    robot.dynamic.zmp.recompute(0)
    controller.zmpDes.value = robot.dynamic.zmp.value
    controller.dcmDes.value = robot.dynamic.zmp.value

    controller.init(dt)
    return controller


def create_dcm_com_controller(Kp, Ki, dt, robot, dcmSignal):
    from math import sqrt
    controller = DcmComController("dcmComCtrl")
    robot.dynamic.com.recompute(0)
    mass = robot.dynamic.data.mass[0]
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g / h)

    controller.Kp.value = Kp
    controller.Ki.value = Ki
    controller.decayFactor.value = 0.2
    controller.mass.value = mass
    controller.omega.value = omega

    controller.ddcomDes.value = [0.0, 0.0, 0.0]

    plug(dcmSignal, controller.dcm)

    robot.dynamic.com.recompute(0)
    controller.comDes.value = robot.dynamic.com.value
    controller.dcmDes.value = (robot.dynamic.com.value[0], robot.dynamic.com.value[1], 0.0)

    controller.init(dt)
    return controller


def create_parameter_server(conf, dt, robot_name='robot'):
    param_server = ParameterServer("param_server")

    # Init should be called before addCtrlMode
    # because the size of state vector must be known.
    param_server.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
        param_server.setNameToId(key, conf.mapJointNameToID[key])

    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
        param_server.setJointLimitsFromId(key, conf.mapJointLimits[key][0], conf.mapJointLimits[key][1])

    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
        param_server.setForceLimitsFromId(key, tuple(conf.mapForceIdToForceLimits[key][0]),
                                          tuple(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
        param_server.setForceNameToForceId(key, conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    param_server.setJointsUrdfToSot(conf.urdftosot)

    # Set the foot frame name
    for key in conf.footFrameNames:
        param_server.setFootFrameName(key, conf.footFrameNames[key])

    # Set IMU hosting joint name
    param_server.setImuJointName(conf.ImuJointName)

    param_server.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ)
    param_server.setRightFootSoleXYZ(conf.rightFootSoleXYZ)

    return param_server


def create_example(robot_name='robot', firstAdd=0., secondAdd=0.):
    example = Example('example')
    example.firstAddend.value = firstAdd
    example.secondAddend.value = secondAdd
    example.init(robot_name)
    return example


def create_dcm_estimator(robot, dt, robot_name='robot'):
    dcm_estimator = DcmEstimator('dcm_estimator')
    dcm_estimator.init(dt, robot_name)
    plug(robot.base_estimator.q, dcm_estimator.q)
    plug(robot.base_estimator.v, dcm_estimator.v)
    return dcm_estimator


def create_distribute_wrench(conf):
    distribute = DistributeWrench('distribute')

    distribute.phase.value = 0
    distribute.rho.value = 0.5

    distribute.setMinPressure(conf.minPressure)
    distribute.frictionCoefficient.value = conf.frictionCoefficient
    distribute.wSum.value = conf.wSum
    distribute.wNorm.value = conf.wNorm
    distribute.wRatio.value = conf.wRatio
    distribute.wAnkle.value = conf.wAnkle

    distribute.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
    distribute.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)

    return distribute


def create_simple_distribute_wrench(name='distribute'):
    distribute = SimpleDistributeWrench(name)

    distribute.phase.value = 0
    distribute.rho.value = 0.5

    return distribute


def create_zmp_estimator(robot, filter=False):
    estimator = SimpleZmpEstimator("zmpEst")
    plug(robot.dynamic.LF, estimator.poseLeft)
    plug(robot.dynamic.RF, estimator.poseRight)
    if filter and hasattr(robot, 'device_filters'):
        plug(robot.device_filters.ft_LF_filter.x_filtered, estimator.wrenchLeft)
        plug(robot.device_filters.ft_RF_filter.x_filtered, estimator.wrenchRight)
    else:
        plug(robot.device.forceLLEG, estimator.wrenchLeft)
        plug(robot.device.forceRLEG, estimator.wrenchRight)

    estimator.init()
    return estimator


def create_ft_calibrator(robot, conf):
    ftc = FtCalibration('ftc')
    ftc.init(robot.name)
    ftc.setRightFootWeight(conf.rfw)
    ftc.setLeftFootWeight(conf.lfw)
    plug(robot.device_filters.ft_RF_filter.x_filtered, ftc.right_foot_force_in)
    plug(robot.device_filters.ft_LF_filter.x_filtered, ftc.left_foot_force_in)
    return ftc


def create_ft_wrist_calibrator(robot, endEffectorWeight, rightOC, leftOC):
    forceCalibrator = FtWristCalibration('forceCalibrator')
    forceCalibrator.init(robot.name)
    forceCalibrator.setRightHandConf(endEffectorWeight, rightOC)
    forceCalibrator.setLeftHandConf(endEffectorWeight, leftOC)
    forceCalibrator.setRemoveWeight(True)
    plug(robot.e2q.quaternion, forceCalibrator.q)
    plug(robot.device_filters.ft_RH_filter.x_filtered, forceCalibrator.rightWristForceIn)
    plug(robot.device_filters.ft_LH_filter.x_filtered, forceCalibrator.leftWristForceIn)
    return forceCalibrator

def set_trigger(robot, on):
    robot.triggerTrajGen.sin.value = 1. if on else 0.

def get_trigger(robot):
    val = robot.triggerTrajGen.sin.value
    if val==1:
        return True
    elif val==0:
        return False
    else:
        raise RuntimeError("Bad trigger")

def load_folder(robot, folder, zmp=False):
    if get_trigger(robot):
        print("Warning: trigger is still active. Not loading folder")
        return
    if folder is not None:
        robot.comTrajGen.playTrajectoryFile(folder + 'CoM.dat')
        robot.lfTrajGen.playTrajectoryFile(folder + 'LeftFoot.dat')
        robot.rfTrajGen.playTrajectoryFile(folder + 'RightFoot.dat')
        if zmp:
            robot.zmpTrajGen.playTrajectoryFile(folder + 'ZMP.dat')
        robot.waistTrajGen.playTrajectoryFile(folder + 'WaistOrientation.dat')
        try:
            robot.rhoTrajGen.playTrajectoryFile(folder + 'Rho.dat')
        except AttributeError:
            pass
        try:
            robot.phaseTrajGen.playTrajectoryFile(folder + 'Phase.dat')
        except AttributeError:
            pass
        try:
            robot.torqueTrajGen.playTrajectoryFile(folder + 'Torques.dat')
        except AttributeError:
            pass

def reload_folder(robot, folder, zmp=False):
    set_trigger(robot, False)
    load_folder(robot, folder, zmp)
    set_trigger(robot, True)


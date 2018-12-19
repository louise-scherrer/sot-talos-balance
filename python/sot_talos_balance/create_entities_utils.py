from dynamic_graph.sot.core.operator import Mix_of_vector
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController
from sot_talos_balance.joint_admittance_controller import JointAdmittanceController

from dynamic_graph.tracer_real_time                           import TracerRealTime
from time                                                     import sleep
from sot_talos_balance.base_estimator                         import BaseEstimator
from sot_talos_balance.madgwickahrs                           import MadgwickAHRS
from dynamic_graph.sot.torque_control.imu_offset_compensation import ImuOffsetCompensation

# python
from sot_talos_balance.utils.filter_utils                     import create_chebi1_checby2_series_filter
from sot_talos_balance.utils.sot_utils                        import Bunch

from dynamic_graph import plug

N_JOINTS = 32;

def create_extend_mix(n_in,n_out):
    assert n_out>n_in
    mix_of_vector = Mix_of_vector( "mix " + str(n_in) + "-" + str(n_out) )

    mix_of_vector.setSignalNumber(3)

    n_diff = n_out-n_in
    mix_of_vector.addSelec(1,0,n_diff)
    mix_of_vector.addSelec(2,n_diff,n_in)

    mix_of_vector.default.value=[0.0]*n_out
    mix_of_vector.signal("sin1").value = [0.0]*n_diff
    mix_of_vector.signal("sin2").value = [2.0]*n_in

    return mix_of_vector

def create_joint_trajectory_generator(dt):
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(N_JOINTS*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_JOINTS);
    return jtg;

def create_config_trajectory_generator(dt):
    N_CONFIG = N_JOINTS + 6
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(N_CONFIG*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_CONFIG);
    return jtg;

def create_com_trajectory_generator(dt,robot):
    comTrajGen = NdTrajectoryGenerator("comTrajGen");
    comTrajGen.initial_value.value = robot.dynamic.com.value
    comTrajGen.trigger.value = 1.0;
    comTrajGen.init(dt, 3);
    return comTrajGen;

def create_joint_controller(Kp):
    controller = JointPositionController("posctrl")
    controller.Kp.value = Kp
    return controller

def create_admittance_controller(Kp,dt,robot):
    controller = JointAdmittanceController("admctrl")
    controller.Kp.value = Kp
    plug(robot.device.state,controller.state)

    mix = create_extend_mix(N_JOINTS,N_JOINTS+6)
    plug(robot.device.ptorque,mix.signal("sin2"))
    plug(mix.sout,controller.tau)

    # plug(robot.device.ptorque,controller.tau)

    controller.tauDes.value = [0.0]*(N_JOINTS+6)
    controller.init(dt, N_JOINTS+6)
    controller.setPosition(robot.device.state.value)
    return controller

def create_imu_offset_compensation(robot, dt):
    imu_offset_compensation = ImuOffsetCompensation('imu_offset_comp');
    imu_offset_compensation.init(dt);
    plug(robot.device.accelerometer, imu_offset_compensation.accelerometer_in);
    plug(robot.device.gyrometer,     imu_offset_compensation.gyrometer_in);
    return imu_offset_compensation;

def create_device_filters(robot, dt):
    filters = Bunch();    
    filters.joints_kin    = create_chebi1_checby2_series_filter("joints_kin", dt, N_JOINTS);
    filters.ft_RF_filter  = create_chebi1_checby2_series_filter("ft_RF_filter", dt, 6);
    filters.ft_LF_filter  = create_chebi1_checby2_series_filter("ft_LF_filter", dt, 6);
    filters.acc_filter    = create_chebi1_checby2_series_filter("acc_filter", dt, 3);
    filters.gyro_filter   = create_chebi1_checby2_series_filter("gyro_filter", dt, 3);
    filters.estimator_kin = create_chebi1_checby2_series_filter("estimator_kin", dt, N_JOINTS);
    
    plug(robot.device.joint_angles,                       filters.estimator_kin.x);  # device.state, device.joint_angles or device.motor_angles ?
    plug(robot.imu_offset_compensation.accelerometer_out, filters.acc_filter.x);
    plug(robot.imu_offset_compensation.gyrometer_out,     filters.gyro_filter.x);
    plug(robot.device.forceRLEG,                          filters.ft_RF_filter.x);
    plug(robot.device.forceLLEG,                          filters.ft_LF_filter.x);
    return filters

def create_be_filters(robot, dt):
    be_filters = Bunch();    
    be_filters.test = create_chebi1_checby2_series_filter("test_filter", dt, N_JOINTS);
    plug(robot.base_estimator.q, be_filters.test.x);
    return be_filters

def create_base_estimator(robot, dt, conf, robot_name="robot"):    
    base_estimator = BaseEstimator('base_estimator');
    base_estimator.init(dt, robot_name);
    plug(robot.device.state,                      base_estimator.joint_positions);  # device.state, device.joint_angles or device.motor_angles ?
    plug(robot.device_filters.ft_LF_filter.x_filtered,   base_estimator.forceLLEG)
    plug(robot.device_filters.ft_RF_filter.x_filtered,   base_estimator.forceRLEG)
    plug(robot.device_filters.ft_LF_filter.dx,           base_estimator.dforceLLEG)
    plug(robot.device_filters.ft_RF_filter.dx,           base_estimator.dforceRLEG)
    plug(robot.device_filters.estimator_kin.dx,          base_estimator.joint_velocities);
    plug(robot.imu_filters.imu_quat,               base_estimator.imu_quaternion);   
    plug(robot.device_filters.gyro_filter.x_filtered,    base_estimator.gyroscope);
    plug(robot.device_filters.acc_filter.x_filtered,     base_estimator.accelerometer);
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses;
    try:
        base_estimator.w_lf_in.value = conf.w_lf_in;
        base_estimator.w_rf_in.value = conf.w_rf_in;
    except:
        pass;
    
    base_estimator.set_imu_weight(conf.w_imu);
    base_estimator.set_stiffness_right_foot(conf.K);
    base_estimator.set_stiffness_left_foot(conf.K);
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
    

    return base_estimator;

def create_imu_filters(robot, dt):
    imu_filter = MadgwickAHRS('imu_filter');
    imu_filter.init(dt);
    plug(robot.device_filters.acc_filter.x_filtered,      imu_filter.accelerometer); # no IMU compensation
    plug(robot.device_filters.gyro_filter.x_filtered,     imu_filter.gyroscope); # no IMU compensation
    return imu_filter;
    
def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);
    
def addSignalsToTracer(tracer, device, outputs):
    for sign in outputs :
         addTrace(tracer,device,sign);
    return
    
def create_tracer(robot,entity,tracer_name, outputs):
    tracer = TracerRealTime(tracer_name)
    tracer.setBufferSize(80*(2**20))
    tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(tracer.name))
    addSignalsToTracer(tracer, entity, outputs)
    return tracer
	
def reset_tracer(device,tracer):
    tracer.stop();
    sleep(0.2);
    tracer.close();
    sleep(0.2);
    tracer.clear();
    sleep(0.2);
    tracer = create_tracer(device, tracer.name);
    return tracer;
    
def dump_tracer(tracer):
    tracer.stop();
    sleep(0.2);
    tracer.dump()
    sleep(0.2);
    tracer.close();
    return


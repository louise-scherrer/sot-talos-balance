''' *********************** USER-PARAMETERS OF BASE ESTIMATOR *********************** '''
# K                   = (4034, 23770, 239018, 707, 502, 936); #HRP2
# K                   = (1., 1., 1., 1., 1., 1.);
K = (1e8, 1e8, 1e8, 1e8, 1e8, 1e8)
std_dev_zmp = 0.02
std_dev_fz = 50.
normal_force_margin = 30.
zmp_margin = 0.002
w_imu = 1.
beta = 0.00329
K_fb_feet_poses = -1.
# gain used for updating foot positions

# --- Real sizes
# RIGHT_FOOT_SIZES    = (0.100, -0.100, 0.06, -0.06)  # pos x, neg x, pos y, neg y size
# LEFT_FOOT_SIZES     = (0.100, -0.100, 0.06, -0.06)  # pos x, neg x, pos y, neg y size

# --- Sizes employed by estimator (weight tuning)
RIGHT_FOOT_SIZES = (0.150, -0.150, 0.11, -0.11)  # pos x, neg x, pos y, neg y size
LEFT_FOOT_SIZES = (0.150, -0.150, 0.11, -0.11)  # pos x, neg x, pos y, neg y size

w_lf_in = 0.
w_rf_in = 1.
# mu                  = 0.3;          # force friction coefficient

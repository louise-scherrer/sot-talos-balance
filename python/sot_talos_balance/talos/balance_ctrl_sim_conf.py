from math import sqrt

from balance_ctrl_conf import *  # noqa

# CONTROLLER GAINS
kp_posture = NJ * (400.0, )  # noqa
kd_posture = NJ * (sqrt(kp_posture[0]), )  # noqa
kd_constr = 0.0 * 2 * sqrt(kp_constr)  # noqa
# constraint derivative feedback gain
kd_com = 0.0 * 2 * sqrt(kp_com)  # noqa
kd_feet = 0.0 * 2 * sqrt(kp_feet)  # noqa

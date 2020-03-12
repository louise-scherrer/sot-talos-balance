from .base_estimator_conf import *  # noqa

# overwrite foot sizes
RIGHT_FOOT_SIZES = (0.100, -0.100, 0.06, -0.06)  # pos x, neg x, pos y, neg y size
LEFT_FOOT_SIZES = (0.100, -0.100, 0.06, -0.06)  # pos x, neg x, pos y, neg y size

minPressure = 15.
frictionCoefficient = 0.7

wSum = 10000.0
wNorm = 10.0
wRatio = 1.0
wAnkle = [1., 1., 1e-4, 1., 1., 1e-4]

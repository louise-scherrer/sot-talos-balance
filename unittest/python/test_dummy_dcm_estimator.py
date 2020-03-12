from numpy.testing import assert_almost_equal as assertApprox

from sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator

dummy = DummyDcmEstimator('dummy')

dummy.mass.value = 1.0
dummy.omega.value = 1.0
dummy.com.value = [1.] * 3
dummy.momenta.value = [2.] * 3

dummy.init()

dummy.dcm.recompute(0)

print(dummy.dcm.value)
assertApprox(dummy.dcm.value, [3.] * 3)

dummy.momenta.value = [3.] * 6
dummy.dcm.recompute(1)
print(dummy.dcm.value)
assertApprox(dummy.dcm.value, [4.] * 3)

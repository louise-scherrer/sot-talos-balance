from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, GainAdaptive, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.operator import Selec_of_vector
from numpy import identity


class MetaTaskJoint(object):
    def __init__(self, dyn, joint, name=None):
        if name is None:
            name = "joint" + str(joint)
        self.dyn = dyn
        self.name = name
        self.joint = joint

        self.feature = FeatureGeneric('feature' + name)
        self.featureDes = FeatureGeneric('featureDes' + name)
        self.gain = GainAdaptive('gain' + name)

        self.selec = Selec_of_vector("selec" + name)
        self.selec.selec(joint, joint + 1)
        plug(dyn.position, self.selec.sin)
        plug(self.selec.sout, self.feature.errorIN)

        robotDim = len(dyn.position.value)
        Id = identity(robotDim)
        J = Id[joint:joint + 1]
        self.feature.jacobianIN.value = matrixToTuple(J)
        self.feature.setReference(self.featureDes.name)

    def plugTask(self):
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self, v):
        self.featureDes.errorIN.value = v


class MetaTaskKineJoint(MetaTaskJoint):
    def __init__(self, dyn, joint, name=None):
        MetaTaskJoint.__init__(self, dyn, joint, name)
        self.task = Task('task' + self.name)
        self.plugTask()

from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, GainAdaptive, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_task_6d import toFlags
from numpy import identity


class MetaTaskConfig(object):
    def __init__(self, dyn, config=None, name="config"):
        self.dyn = dyn
        self.name = name
        self.config = config

        self.feature = FeatureGeneric('feature' + name)
        self.featureDes = FeatureGeneric('featureDes' + name)
        self.gain = GainAdaptive('gain' + name)

        plug(dyn.position, self.feature.errorIN)
        robotDim = dyn.getDimension()
        self.feature.jacobianIN.value = matrixToTuple(identity(robotDim))
        self.feature.setReference(self.featureDes.name)
        if config is not None:
            self.feature.selec.value = toFlags(self.config)

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


class MetaTaskKineConfig(MetaTaskConfig):
    def __init__(self, dyn, config=None, name="config"):
        MetaTaskConfig.__init__(self, dyn, config, name)
        self.task = Task('task' + name)
        self.plugTask()

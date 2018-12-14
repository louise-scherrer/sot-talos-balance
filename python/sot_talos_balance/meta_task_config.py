from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph import plug
from dynamic_graph.sot.core import *
# from dynamic_graph.sot.core.meta_tasks import setGain
# from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate
from numpy import matrix, identity, zeros, eye


class MetaTaskConfig(object):
    nbDof = None

    def __init__(self,dyn,config,name="joint"):
        self.dyn=dyn
        self.name=name
        self.config = config

        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.gain = GainAdaptive('gain'+name)

        plug(dyn.position,self.feature.errorIN)
        robotDim = len(dyn.position.value)
        self.feature.jacobianIN.value = matrixToTuple( identity(robotDim) )
        self.feature.setReference(self.featureDes.name)
        self.feature.selec.value = toFlags(self.config)

    def plugTask(self):
        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self,v):
        self.featureDes.errorIN.value = v

class MetaTaskKineConfig(MetaTaskConfig):
    def __init__(self,dyn,config,name="joint"):
        MetaTaskConfig.__init__(self,dyn,config,name)
        self.task = Task('task'+name)
        self.plugTask()
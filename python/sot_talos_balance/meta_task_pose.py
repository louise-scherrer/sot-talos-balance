from dynamic_graph import plug
from dynamic_graph.sot.core import GainAdaptive, Task
from dynamic_graph.sot.core.feature_pose import FeaturePose


class MetaTaskPose(object):
    def opPointExist(self, opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint, self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J' + opPoint, self.dyn.signals())
        return len(sigsP) == 1 & len(sigsJ) == 1

    def defineDynEntities(self, dyn):
        self.dyn = dyn

    def createOpPoint(self, opPoint, opPointRef='right-wrist'):
        self.opPoint = opPoint
        if self.opPointExist(opPoint):
            return
        self.dyn.createOpPoint(opPoint, opPointRef)

    def createFeature(self):
        self.feature = FeaturePose('feature' + self.name)
        self.feature.selec.value = '111111'

    def createTask(self):
        self.task = Task('task' + self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain' + self.name)
        self.gain.set(0.1, 0.1, 125e3)

    def plugEverything(self):
        plug(self.dyn.signal(self.opPoint), self.feature.oMjb)
        plug(self.dyn.signal('J' + self.opPoint), self.feature.jbJjb)
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    def keep(self):
        self.feature.faMfb.recompute(self.dyn.position.time)
        self.feature.faMfbDes.value = self.feature.faMfb.value

    def __init__(self, name, dyn, opPoint, opPointRef='right-wrist'):
        self.name = name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint, opPointRef)
        self.createFeature()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def ref(self):
        return self.feature.faMfbDes.value

    @ref.setter
    def ref(self, m):
        self.feature.faMfbDes.value = m

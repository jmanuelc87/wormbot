from abc import ABCMeta, abstractmethod


class Listener:
    __metaclass__ = ABCMeta


class AxisListener(Listener):

    @abstractmethod
    def onAxisMoveAction(self, x, y):
        pass


class EventListener(Listener):

    @abstractmethod
    def onButtonDown(self):
        pass

    @abstractmethod
    def onButtonUp(self):
        pass

    @abstractmethod
    def onButtonPress(self):
        pass

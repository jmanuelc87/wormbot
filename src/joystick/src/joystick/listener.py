from abc import ABC, abstractmethod


class Listener(ABC):
    pass


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

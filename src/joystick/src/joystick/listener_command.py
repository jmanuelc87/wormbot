from abc import ABCMeta, abstractmethod
from rospy import loginfo, loginfo_throttle, logerr


class Command(object):

    @abstractmethod
    def handle(self, request):
        pass

    @abstractmethod
    def set_next(self, command):
        pass


class AbstractCommand(Command):

    def __init__(self):
        self._next_command = None

    def set_next(self, command):
        self._next_command = command
        return command

    def next(self, request):
        if self._next_command:
            return self._next_command.handle(request)

    def handle(self, request):
        pass


class IncreaseSpeedCommand(AbstractCommand):

    def __init__(self, speed, speed_limits_x=[0.0, 2.0], speed_limits_y=[0.0, 1.2]):
        self.speed_limits_x = speed_limits_x
        self.speed_limits_y = speed_limits_y
        self.speed = speed

    def handle(self, request):
        if self.speed['angular'] - 0.001 <= 0.0:
            request.angular.z = request.angular.z * 0.0
        else:
            request.angular.z = request.angular.z * self.speed['angular']

        if self.speed['linear'] - 0.001 <= 0.0:
            request.linear.x = request.linear.x * 0.0
        else:
            request.linear.x = request.linear.x * self.speed['linear']

        return self.next(request)

    def increase(self):
        if self.speed['angular'] <= self.speed_limits_x[1]:
            self.speed['angular'] += 0.3

        if self.speed['angular'] >= self.speed_limits_x[1]:
            self.speed['angular'] = self.speed_limits_x[1]

        if self.speed['linear'] <= self.speed_limits_y[1]:
            self.speed['linear'] += 0.1

        if self.speed['linear'] >= self.speed_limits_y[1]:
            self.speed['linear'] = self.speed_limits_y[1]


class DecreaseSpeedCommand(AbstractCommand):

    def __init__(self, speed, speed_limits_x=[0.0, 5.0], speed_limits_y=[0.0, 1.0]):
        self.speed_limits_x = speed_limits_x
        self.speed_limits_y = speed_limits_y
        self.speed = speed

    def handle(self, request):
        if self.speed['angular'] - 0.001 <= 0.0:
            request.angular.z = request.angular.z * 0.0
        else:
            request.angular.z = request.angular.z * self.speed['angular']

        if self.speed['linear'] - 0.001 <= 0.0:
            request.linear.x = request.linear.x * 0.0
        else:
            request.linear.x = request.linear.x * self.speed['linear']

        return self.next(request)

    def decrease(self):
        if self.speed['angular'] >= self.speed_limits_x[0]:
            self.speed['angular'] -= 0.3
        
        if self.speed['angular'] <= self.speed_limits_x[0]:
            self.speed['angular'] = self.speed_limits_x[0]

        if self.speed['linear'] >= self.speed_limits_y[0]:
            self.speed['linear'] -= 0.1
        
        if self.speed['linear'] <= self.speed_limits_y[0]:
            self.speed['linear'] = self.speed_limits_y[0]


class PositionMoveListener(AbstractCommand):

    def handle(self, request, x, y):
        request.angular.z = x
        request.linear.x = y
        return self.next(request)


class BrakeCommand(AbstractCommand):

    def __init__(self):
        self.stop = False

    def handle(self, request):
        if self.stop:
            request.angular.z = 0.0
            request.linear.x = 0.0
            return request
        else:
            return request

    def set_stop(self, stop):
        self.stop = stop


class CommandInvoker:

    def __init__(self):
        self.position = PositionMoveListener()
        speed = {'angular': 0.0, 'linear': 0.0}
        self.increaseCmd = IncreaseSpeedCommand(speed)
        self.decreaseCmd = DecreaseSpeedCommand(speed)
        self.brake = BrakeCommand()

        self.position.set_next(self.increaseCmd).set_next(self.decreaseCmd).set_next(self.brake)

    def move(self, request, x, y):
        return self.position.handle(request, x, y)

    def increase(self):
        self.increaseCmd.increase()

    def decrease(self):
        self.decreaseCmd.decrease()

    def stop(self, stop):
        self.brake.set_stop(stop)

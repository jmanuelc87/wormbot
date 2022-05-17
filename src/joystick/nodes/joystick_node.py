import math
import rospy

from rospy import loginfo, loginfo_throttle, logerr
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from joystick import ListenerController, ControlDefinition, EventListener, AxisListener, CommandInvoker


rospy.init_node("joystick_node")

TOPIC = '/wormbot/movement_controller/cmd_vel'
topic = rospy.Publisher(TOPIC, Twist, queue_size=10)

invoker = CommandInvoker()


class LeftThumbListener(AxisListener):

    def onAxisMoveAction(self, x, y):
        msg = invoker.move(Twist(), x, y)
        return msg


class ButtonAListener(EventListener):

    def onButtonDown(self):
        return invoker.stop(True)

    def onButtonUp(self):
        return invoker.stop(False)

    def onButtonPress(self):
        pass


class LeftShoulderListener(EventListener):

    def onButtonDown(self):
        pass

    def onButtonUp(self):
        pass

    def onButtonPress(self):
        invoker.decrease()


class RightShoulderListener(EventListener):

    def onButtonDown(self):
        pass

    def onButtonUp(self):
        pass

    def onButtonPress(self):
        invoker.increase()


ns = rospy.get_namespace()
control = ListenerController("/wormbot/joy", topic)
control.addListener(ControlDefinition.LEFT_THUMB, LeftThumbListener())
control.addListener(ControlDefinition.LEFT_SHOULDER, LeftShoulderListener())
control.addListener(ControlDefinition.RIGHT_SHOULDER, RightShoulderListener())
control.addListener(ControlDefinition.BUTTON_A, ButtonAListener())
rospy.spin()

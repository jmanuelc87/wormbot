import math
import rospy

from rospy import loginfo, logerr
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from joystick import Controller, ControlDefinition, EventListener, AxisListener




FACTOR = 5.0
brake = False


class PositionMoveListener(AxisListener):

    def onAxisMoveAction(self, x, y):
        global brake
        if not brake:
            twist = Twist()
            twist.angular.z = x * FACTOR
            twist.linear.x = y
            return twist
        else:
            return Twist()


class AccelerationListener(AxisListener):
    
    def onAxisMoveAction(self, l, r):
        global brake
        if not brake:
            twist = Twist()
            if l > 0.0 and r > 0.0:
                pass
            elif l == 0.0 and r == 0.0:
                twist.angular.z = 0.0
            elif l > 0.0 and r == 0.0:
                twist.angular.z = -1 * l * FACTOR
            elif r > 0.0 and l == 0.0:
                twist.angular.z = r * FACTOR
            
            return twist
        else:
            return Twist()


class BrakeListener(EventListener):

    def onButtonDown(self):
        global brake
        brake = True
        return Twist()

    def onButtonUp(self):
        global brake
        brake = False
        return Twist()

    def onButtonPress(self):
        pass


if __name__ == '__main__':
    rospy.init_node("joystick_node")

    ns = rospy.get_namespace()

    TOPIC = '/wormbot/movement_controller/cmd_vel'

    pub = rospy.Publisher(TOPIC, Twist, queue_size=100)

    control = Controller("/wormbot/joy", pub)
    control.addListener(ControlDefinition.LEFT_THUMB, PositionMoveListener())
    control.addListener(ControlDefinition.TRIGGERS, AccelerationListener())
    control.addListener(ControlDefinition.BUTTON_A, BrakeListener())

    rospy.spin()

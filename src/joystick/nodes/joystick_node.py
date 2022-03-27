
import math
import rospy
from rospy.core import logdebug

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from joystick import Controller, ControlDefinition, EventListener, AxisListener



class PositionMoveListener(AxisListener):

    def __init__(self):
        self.publisher = rospy.Publisher('/wormbot/movement_controller/cmd_vel', Twist, queue_size=10)

    def onAxisMoveAction(self, x, y):
        twist = Twist()
        twist.angular.z = y * 8.0
        twist.linear.x = x

        self.publisher.publish(twist)


def main():
    rospy.init_node("joystick_node", log_level=rospy.DEBUG)

    ns = rospy.get_namespace()

    control = Controller("/walleye/joy")
    control.addListener(ControlDefinition.LEFT_AXIS, PositionMoveListener())

    rospy.spin()


if __name__ == '__main__':
    main()

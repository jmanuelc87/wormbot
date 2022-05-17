import rospy

from enum import Enum
from joystick import Listener
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from joystick import AxisListener, EventListener


HIGH = 1.0
LOW = 0.0


class ControlDefinition(Enum):
    LEFT_THUMB = (0, 1),
    RIGHT_THUMB = (2, 3),

    TRIGGERS = (4, 5),

    LEFT_SHOULDER = 10,
    RIGHT_SHOULDER = 11,

    BUTTON_A = 4,
    BUTTON_B = 5,
    BUTTON_X = 6,
    BUTTON_Y = 7


class ListenerController(object):

    def __init__(self, topic, pub):
        self.subscriber = rospy.Subscriber(
            topic, Joy, callback=self.__control_subscriber)
        self.listeners = {}
        self.current_seq = 0
        self.pub = pub
        self.message = None

    def __del__(self):
        self.subscriber.unregister()

    def addListener(self, definition, listener):
        message = Joy()
        message.buttons = [0 for i in range(12)]
        message.axes = [0 for i in range(12)]

        self.listeners[definition] = [listener, message, False, False]

    def __is_new(self, current, last):
        return current.header.seq > last.header.seq

    def __perform_axis_listener(self, message, key):
        curr_listener = self.listeners[key][0]
        last_msg = self.listeners[key][1]
        x = message.axes[key.value[0][0]]
        y = message.axes[key.value[0][1]]

        self.message = curr_listener.onAxisMoveAction(x, y)

    def __perform_event_listener(self, message, key):
        curr_listener = self.listeners[key][0]
        last_msg = self.listeners[key][1]

        if last_msg.buttons[key.value[0]] != message.buttons[key.value[0]] and message.buttons[key.value[0]] == HIGH:
            curr_listener.onButtonDown()
            self.listeners[key][2] = True

        if last_msg.buttons[key.value[0]] != message.buttons[key.value[0]] and message.buttons[key.value[0]] == LOW:
            curr_listener.onButtonUp()
            self.listeners[key][3] = True

        if self.listeners[key][2] and self.listeners[key][3]:
            curr_listener.onButtonPress()
            self.listeners[key][2] = False
            self.listeners[key][3] = False

    def __publish(self, message):
        if message:
            self.pub.publish(message)
        else:
            self.pub.publish(Twist())

    def __control_subscriber(self, message):
        for key in self.listeners.keys():
            current_listener = self.listeners[key]

            if self.__is_new(message, current_listener[1]):
                # message is a recent one, test for the type of listener
                if isinstance(current_listener[0], AxisListener):
                    self.__perform_axis_listener(message, key)
                elif isinstance(current_listener[0], EventListener):
                    self.__perform_event_listener(message, key)
                else:
                    raise RuntimeError("Not defined...")

            current_listener[1] = message

        self.__publish(self.message)

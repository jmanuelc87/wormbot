#!/usr/bin/env python
import time
import rospy
import threading


from board import MotorDriverI2C as MotorDriver

from drivers.msg import SpeedMessage
from drivers.srv import SpeedCommand


lock = threading.Lock()

motor_left_speed = 0
motor_right_speed = 0

# Namespace
ns = rospy.get_namespace()

# motor driver
driver = MotorDriver(1, 0x10)

l = driver.detect()
rospy.loginfo("Board list conform: %s", l)

# Start Node
rospy.init_node("driver_node", log_level=rospy.DEBUG)


def print_board_status():
    if driver.last_operate_status == driver.STA_OK:
        rospy.logdebug("driver status: everything ok")
    elif driver.last_operate_status == driver.STA_ERR:
        rospy.logdebug("driver status: unexpected error")
    elif driver.last_operate_status == driver.STA_ERR_DEVICE_NOT_DETECTED:
        rospy.logdebug("driver status: device not detected")
    elif driver.last_operate_status == driver.STA_ERR_PARAMETER:
        rospy.logdebug("driver status: parameter error, last operate no effective")
    elif driver.last_operate_status == driver.STA_ERR_SOFT_VERSION:
        rospy.logdebug("driver status: unsupport driver framware version")


def set_motor_speed(message):
    global motor_left_speed
    global motor_right_speed

    lock.acquire()
    motor_left_speed = message.speedL;
    motor_right_speed = message.speedR;
    lock.release()


def get_motor_speed(req):
    speeds = driver.get_encoder_speed(MotorDriver.ALL)

    res = SpeedCommand()
    res.speedL = speeds[0]
    res.speedR = speeds[1]

    return res


def on_shutdown():
    driver.motor_stop(MotorDriver.ALL)
    rospy.loginfo("Bye Bye!!!")


# Create set motor speed service
rospy.Subscriber(ns + "drivers/set_motor_speed", SpeedMessage, callback=set_motor_speed)
rospy.Service(ns + "drivers/get_motor_speed", SpeedCommand, get_motor_speed)

rospy.on_shutdown(on_shutdown)

rospy.loginfo("Connection to motor board beginning")

while driver.begin() != MotorDriver.STA_OK:    # Board begin and check board status
    print_board_status()
    rospy.loginfo("board begin failed")
    time.sleep(2)
rospy.loginfo("board begin success")

driver.set_encoder_enable(MotorDriver.ALL)

driver.set_encoder_reduction_ratio(MotorDriver.ALL, 60)

driver.set_motor_pwm_frequency(1000)

speeds = driver.get_encoder_speed(MotorDriver.ALL)

get_motor_speed.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

rospy.loginfo("Starting main loop...")

rate = rospy.Rate(30)

while not rospy.is_shutdown():

    lock.acquire()
    if motor_left_speed > 0:
        driver.motor_movement([MotorDriver.M1], MotorDriver.CW, (motor_left_speed / 160) * 100)
    elif motor_left_speed < 0:
        driver.motor_movement([MotorDriver.M1], MotorDriver.CCW, (-motor_left_speed / 160) * 100)
    else:
        driver.motor_stop([MotorDriver.M1])

    if motor_right_speed > 0:
        driver.motor_movement([MotorDriver.M2], MotorDriver.CCW, (motor_right_speed / 160) * 100)
    elif motor_right_speed < 0:
        driver.motor_movement([MotorDriver.M2], MotorDriver.CW, (-motor_right_speed / 160) * 100)
    else:
        driver.motor_stop([MotorDriver.M2])
    lock.release()

    rate.sleep()

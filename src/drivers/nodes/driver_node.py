#!/usr/bin/env python
import time
import rospy
import threading


from simple_pid import PID

from board import MotorDriverI2C as MotorDriver

from drivers.msg import Speed
from drivers.cfg import PIDLimitsConfig

from dynamic_reconfigure.server import Server

# PID Constants
pid_values = {
    "Kp": 0.3522,
    "Ki": 0.2317,
    "Kd": 0.0798
}

lock = threading.Lock()

motor_left_speed = 0
motor_right_speed = 0

spins = []

# Namespace
ns = rospy.get_namespace()

# PID Object
pidL = PID(Kp=pid_values["Kp"], Ki=pid_values["Ki"], Kd=pid_values["Kd"])
pidR = PID(Kp=pid_values["Kp"], Ki=pid_values["Ki"], Kd=pid_values["Kd"])

# motor driver
driver = MotorDriver(1, 0x10)

l = driver.detect()
rospy.loginfo("Board list conform: %s", l)

# Start Node
rospy.init_node("driver_node")


def print_board_status():
    if driver.last_operate_status == driver.STA_OK:
        print("driver status: everything ok")
    elif driver.last_operate_status == driver.STA_ERR:
        print("driver status: unexpected error")
    elif driver.last_operate_status == driver.STA_ERR_DEVICE_NOT_DETECTED:
        print("driver status: device not detected")
    elif driver.last_operate_status == driver.STA_ERR_PARAMETER:
        print("driver status: parameter error, last operate no effective")
    elif driver.last_operate_status == driver.STA_ERR_SOFT_VERSION:
        print("driver status: unsupport driver framware version")


def reconfigure_callback(config, level):
    pidL.Kp = config.Kp
    pidL.Ki = config.Ki
    pidL.Kd = config.Kd
    pidL.auto_mode = config.enable
    pidR.Kp = config.Kp
    pidR.Ki = config.Ki
    pidR.Kd = config.Kd
    pidR.auto_mode = config.enable
    # TODO: plot values
    return config


def set_motor_speed(message):
    rospy.loginfo("%s", message)

    lock.acquire()
    spins = []
    if message.speedL > 0:
        motor_left_speed = int((message.speedL / 160) * 100)
        spins.append(MotorDriver.CW)
        rospy.loginfo("move forward")
    elif message.speedL < 0:
        motor_left_speed = int((-message.speedL / 160) * 100)
        spins.append(MotorDriver.CCW)
        rospy.loginfo("move backward")
    else:
        motor_left_speed = 0
        spins.append(MotorDriver.STOP)
        rospy.loginfo("Stop")

    if message.speedR > 0:
        motor_right_speed = int((message.speedR / 160) * 100)
        spins.append(MotorDriver.CCW)
        rospy.loginfo("move forward")
    elif message.speedR < 0:
        motor_right_speed = int((-message.speedR / 160) * 100)
        spins.append(MotorDriver.CW)
        rospy.loginfo("move backward")
    else:
        motor_right_speed = 0
        spins.append(MotorDriver.STOP)
        rospy.loginfo("Stop")
    lock.release()

    rospy.loginfo("Speed %s, %s, Spin: %s", motor_left_speed, motor_right_speed, spins)


def on_shutdown():
    driver.motor_stop(MotorDriver.ALL)
    rospy.loginfo("Bye Bye!!!")


# Create Reconfigure Configure Server
server = Server(PIDLimitsConfig, reconfigure_callback)

# Create set motor speed service
rospy.Subscriber(ns + "drivers/set_motor_speed", Speed, callback=set_motor_speed)
speedPublisher = rospy.Publisher(ns + "drivers/get_motor_speed", Speed, queue_size=10, latch=True)

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

speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

rospy.loginfo("Starting main loop...")

rate = rospy.Rate(24)

while not rospy.is_shutdown():
    lock.acquire()
    # TODO: SETUP PID
    driver.motor_movement(MotorDriver.ALL, spins, [motor_left_speed, motor_right_speed])
    lock.release()

    speeds = driver.get_encoder_speed(MotorDriver.ALL)

    speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

    rate.sleep()

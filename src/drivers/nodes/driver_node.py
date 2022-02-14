#!/usr/bin/python
import time

import rospy

from simple_pid import PID

from board import MotorDriverI2C as MotorDriver
from drivers.msg import Speed, Duty
from drivers.cfg import PIDLimitsConfig
from dynamic_reconfigure.server import Server

# PID Constants
pid_values = {
    "Kp": 0.3522,
    "Ki": 0.2317,
    "Kd": 0.0798
}

motor_duty = {
    "mL": 0,
    "mR": 0
}

spins = []

# Namespace
ns = rospy.get_namespace()

# PID Object
pidL = PID(Kp=pid_values["Kp"], Ki=pid_values["Ki"], Kd=pid_values["Kd"])
pidR = PID(Kp=pid_values["Kp"], Ki=pid_values["Ki"], Kd=pid_values["Kd"])

# motor driver
driver = MotorDriver(1, 0x10)

l = driver.detecte()
rospy.loginfo("Board list conform: %s", l)


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


def set_motor_duty(message):
    motor_duty["mL"] = message.dutyL
    motor_duty["mR"] = message.dutyR

    if motor_duty["mL"] > 0:
        spins.append(MotorDriver.CCW)
    elif motor_duty["mL"] < 0:
        spins.append(MotorDriver.CW)
    else:
        spins.append(MotorDriver.STOP)

    if motor_duty["mR"] > 0:
        spins.append(MotorDriver.CW)
    elif motor_duty["mR"] < 0:
        spins.append(MotorDriver.CCW)
    else:
        spins.append(MotorDriver.STOP)

    rospy.loginfo("%s %s", motor_duty, spins)


def on_shutdown():
    driver.motor_stop(MotorDriver.ALL)


# Start Node
rospy.init_node("driver_node")

# Create Reconfigure Configure Server
server = Server(PIDLimitsConfig, reconfigure_callback)

# Create set motor speed service
rospy.Subscriber(ns + "drivers/set_motor_duty", Duty, set_motor_duty)
speedPublisher = rospy.Publisher(ns + "drivers/get_motor_speed", Speed, queue_size=1)

rate = rospy.Rate(15)

rospy.on_shutdown(on_shutdown)

rospy.loginfo("Connection to motor board beginning")

while driver.begin() != MotorDriver.STA_OK:    # Board begin and check board status
    print_board_status()
    rospy.loginfo("board begin failed")
    time.sleep(2)
rospy.loginfo("board begin success")

time.sleep(2)

rospy.loginfo("Motor encoder is being enabled")

driver.set_encoder_enable(MotorDriver.ALL)

time.sleep(2)

rospy.loginfo("Setting motor reduction ratio")

driver.set_encoder_reduction_ratio(MotorDriver.ALL, 60)

time.sleep(2)

rospy.loginfo("Setting PWM frequency")

driver.set_motor_pwm_frequency(1000)

time.sleep(2)

speeds = driver.get_encoder_speed(MotorDriver.ALL)

speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

while not rospy.is_shutdown():
    driver.motor_movement(MotorDriver.ALL, spins, [motor_duty["mL"], motor_duty["mR"]])

    speeds = driver.get_encoder_speed(MotorDriver.ALL)

    speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

    rate.sleep()

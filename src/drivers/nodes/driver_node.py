#!/usr/bin/python
import time

import rospy

from simple_pid import PID

from board import MotorDriver
from board import SpinEnum
from drivers.msg import Speed
from drivers.srv import Duty
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
driver = MotorDriver()


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
        spins.append(SpinEnum.CCW.value)
    elif motor_duty["mL"] < 0:
        spins.append(SpinEnum.CW.value)
    else:
        spins.append(SpinEnum.STOP.value)

    if motor_duty["mR"] > 0:
        spins.append(SpinEnum.CW.value)
    elif motor_duty["mR"] < 0:
        spins.append(SpinEnum.CCW.value)
    else:
        spins.append(SpinEnum.STOP.value)

    rospy.loginfo("%s %s", motor_duty, spins)


# Start Node
rospy.init_node("driver_node")

# Create Reconfigure Configure Server
server = Server(PIDLimitsConfig, reconfigure_callback)

# Create set motor speed service
rospy.Service(ns + "drivers/set_motor_duty", Duty, set_motor_duty)
speedPublisher = rospy.Publisher(ns + "drivers/get_motor_speed", Speed, queue_size=1)

rate = rospy.Rate(15)

rospy.on_shutdown(driver.stop)

rospy.loginfo("Connection to motor board beginning")

driver.begin()

time.sleep(2)

rospy.loginfo("Motor encoder is being enabled")

driver.enable()

time.sleep(2)

rospy.loginfo("Setting motor reduction ratio")

driver.reduction_ratio(60)

time.sleep(2)

rospy.loginfo("Setting PWM frequency")

driver.pwm_frequency(1000)

time.sleep(2)

speeds = driver.speed()

speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

while not rospy.is_shutdown():
    driver.move([motor_duty["mL"], motor_duty["mR"]], spins)

    speeds = driver.speed()

    speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

    rate.sleep()

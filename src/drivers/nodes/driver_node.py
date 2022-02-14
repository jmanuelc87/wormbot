#!/usr/bin/python
import time

import rospy

from simple_pid import PID

from board import MotorDriver
from board import SpinEnum
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
    "mR": 0,
    "orientationL": SpinEnum.CW,
    "orientationR": SpinEnum.CCW
}

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

    pidL.setpoint = message.dutyL
    pidR.setpoint = message.dutyR

    motor_duty["orientationL"] = orientation(message.orientationL)
    motor_duty["orientationR"] = orientation(message.orientationR)


def orientation(message):
    if message == 1:
        return SpinEnum.CW
    elif message == 2:
        return SpinEnum.CCW
    else:
        raise RuntimeError("Can't map the orientation")


# Create Reconfigure Configure Server
server = Server(PIDLimitsConfig, reconfigure_callback)

# Start Node
rospy.init_node("driver_node")

# Create set motor speed service
dutySubscriber = rospy.Subscriber(ns + "/drivers/set_motor_duty", Duty, set_motor_duty)
speedPublisher = rospy.Publisher(ns + "/drivers/get_motor_speed", Speed)

rate = rospy.Rate(30)

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
    control_duty1 = pidL(motor_duty["mL"])
    control_duty2 = pidR(motor_duty["mR"])

    driver.move([control_duty1, control_duty2], [bytes(motor_duty["orientationL"]), bytes(motor_duty["orientationR"])])

    speeds = driver.speed()

    speedPublisher.publish(Speed(speedL=speeds[0], speedR=speeds[1]))

    rate.sleep()

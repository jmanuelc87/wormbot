from motor import I2C
from motor import Motor


class MotorDriver:
    __BUS = 1
    __ADDRESS = 0x10

    def __init__(self):
        self._i2c = I2C(self.__BUS, self.__ADDRESS)
        self.motors = [Motor(self._i2c, mid) for mid in range(1, 3)]

    def begin(self):
        for motor in self.motors:
            motor.begin()

    def move(self, duties, orientations):
        if len(duties) == 2 and len(orientations) == 2:
            for zipped in zip(self.motors, duties, orientations):
                zipped[0].move(zipped[1], zipped[2])

    def speed(self):
        speeds = []
        for motor in self.motors:
            speed = motor.encoder_speed()
            speeds.append(speed)
        return speeds

    def stop(self):
        for motor in self.motors:
            motor.stop()

    def enable(self):
        for motor in self.motors:
            motor.enable_encoder()

    def reduction_ratio(self, ratio):
        for motor in self.motors:
            motor.encoder_reduction_ratio(ratio)

    def pwm_frequency(self, frequency):
        for motor in self.motors:
            motor.pwm_frequency(frequency)

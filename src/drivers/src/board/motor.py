import enum
import time
import smbus
import logging


class MotorEnum(enum.Enum):
    """ Enum motor ID """
    M1 = 0x01
    M2 = 0x02
    ALL = 0xFFFFFFFF


class SpinEnum(enum.Enum):
    CW = 0x01  # clockwise
    CCW = 0x02  # counterclockwise
    STOP = 0x05


class _Status(enum.Enum):
    STA_OK = 0x00
    STA_ERR = 0x01
    STA_ERR_DEVICE_NOT_DETECTED = 0x02
    STA_ERR_SOFT_VERSION = 0x03
    STA_ERR_PARAMETER = 0x04


class I2C:

    def __init__(self, bus, addr):
        self._bus = smbus.SMBus(bus)
        self._addr = addr
        self.last_operate_status = _Status.STA_OK

    def write_bytes(self, reg, buf):
        try:
            self._bus.write_i2c_block_data(self._addr, reg, buf)
            self.last_operate_status = _Status.STA_OK
        except RuntimeError as err:
            logging.error("Error while writing to device: %s", err)
            self.last_operate_status = _Status.STA_ERR_DEVICE_NOT_DETECTED

    def read_bytes(self, reg, length):
        try:
            result = self._bus.read_i2c_block_data(self._addr, reg, length)
            self.last_operate_status = _Status.STA_OK
            return result
        except RuntimeError as err:
            logging.error("Error while reading from device: %s", err)
            self.last_operate_status = _Status.STA_ERR_DEVICE_NOT_DETECTED
            return None


class Motor:
    _STEPPER_COUNT = 1
    _MOTOR_COUNT = 2

    _REG_SLAVE_ADDR = 0x00
    _REG_PID = 0x01
    _REG_PVD = 0x02
    _REG_CTRL_MODE = 0x03
    _REG_ENCODER1_EN = 0x04
    _REG_ENCODER1_SPPED = 0x05
    _REG_ENCODER1_REDUCTION_RATIO = 0x07
    _REG_ENCODER2_EN = 0x09
    _REG_ENCODER2_SPEED = 0x0a
    _REG_ENCODER2_REDUCTION_RATIO = 0x0c
    _REG_MOTOR_PWM = 0x0e
    _REG_MOTOR1_ORIENTATION = 0x0f
    _REG_MOTOR1_SPEED = 0x10
    _REG_MOTOR2_ORIENTATION = 0x12
    _REG_MOTOR2_SPEED = 0x13

    _REG_DEF_PID = 0xdf
    _REG_DEF_VID = 0x10

    def __init__(self, i2c, motor_id):
        self._id = motor_id
        self._i2c = i2c

    def begin(self):
        pid = self._i2c.read_bytes(self._REG_PID, 1)
        vid = self._i2c.read_bytes(self._REG_PVD, 1)
        if self._i2c.last_operate_status == _Status.STA_OK:
            if pid[0] != self._REG_DEF_PID:
                return False
            else:
                # Set control mode to DC Motor
                self._i2c.write_bytes(self._REG_CTRL_MODE, [0x00])
                self.stop()
                self.disable_encoder()
        return True

    def disable_encoder(self):
        self._i2c.write_bytes(self._REG_ENCODER1_EN + 5 * (self._id - 1), [0x00])
        return self._i2c.last_operate_status == _Status.STA_OK

    def enable_encoder(self):
        self._i2c.write_bytes(self._REG_ENCODER1_EN + 5 * (self._id - 1), [0x01])
        return self._i2c.last_operate_status == _Status.STA_OK

    def encoder_reduction_ratio(self, ratio):
        reduction_ratio = int(ratio)
        if reduction_ratio >= 1 or reduction_ratio < 2000:
            self._i2c.write_bytes(self._REG_ENCODER1_REDUCTION_RATIO + 5 * (self._id - 1),
                                  [reduction_ratio >> 8, reduction_ratio & 0xFF])
            return self._i2c.last_operate_status == _Status.STA_OK
        return False

    def encoder_speed(self):
        result = self._i2c.read_bytes(self._REG_ENCODER1_SPPED + 5 * (self._id - 1), 2)
        s = (result[0] << 8) | result[1]
        if s & 0x8000:
            s = - (0x10000 - s)
        return s

    def pwm_frequency(self, frequency):
        if 100 <= frequency <= 12750:
            frequency = int(frequency / 50)
            self._i2c.write_bytes(self._REG_MOTOR_PWM, [frequency])
            time.sleep(0.1)
            return self._i2c.last_operate_status == _Status.STA_OK
        return False

    def move(self, speed, orientation):
        if orientation in [SpinEnum.CW, SpinEnum.CCW] and (0.0 <= speed <= 100.0):
            reg = self._REG_MOTOR1_ORIENTATION + (self._id - 1) * 3
            self._i2c.write_bytes(reg, [orientation])
            self._i2c.write_bytes(reg + 1, [int(speed), int((speed * 10) % 10)])
            return self._i2c.last_operate_status == _Status.STA_OK
        return False

    def stop(self):
        self._i2c.write_bytes(self._REG_MOTOR1_ORIENTATION + 3 * (self._id - 1), [SpinEnum.STOP])
        return self._i2c.last_operate_status == _Status.STA_OK

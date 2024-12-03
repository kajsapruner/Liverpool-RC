import sys
from math import pi
from machine import Pin, PWM
from time import sleep

class Motor:

    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, slp_pin):
        # set pins
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._slp_pin = Pin(slp_pin, Pin.OUT)  # Sleep pin
        self._pwm_pin.freq(1000)
        # stop motor
        self.disable()
        # interrupts
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._inc_counts)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._inc_counts)
        # variables
        self.encoder_counts = 0

        # Stop motor initially
        self.stop()

    def _inc_counts(self, pin):
        self.encoder_counts += 1

    def stop(self):
        # Stop the motor and put to sleep
        self._pwm_pin.duty_u16(0)
        # self._slp_pin.value(0)  # Sleep motor

    def enable(self):
        self._slp_pin.value(1)

    def disable(self):
        self._pwm_pin.duty_ns(0)
        self._slp_pin.value(0)

    def forward(self, duty=1.0):
        # Move motor forward
        # self._slp_pin.value(1)  # Wake up motor
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty*65536))

    def backward(self, duty=1.0):
        # Move motor backward
        # self._slp_pin.value(1)  # Wake up motor
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty*65536))

    # def set_velocity(self, velocity):
    #     # For direct control, you can set desired velocity here
    #     self.target_velocity = velocity

    def reset_encoder(self):
        # Reset encoder counts
        self.encoder_counts = 0


if __name__ == "__main__":
    # Initialize your robot instance
    from machine import freq
    freq(200_000_000)
    # constants
    WHEEL_RADIUS = 0.0495  # m
    WHEEL_SEPARATION = 0.295  # m
    CPR = 64
    GEAR_RATIO = 70

    lm = Motor(3, 2, 14, 15, 4)
    rm = Motor(7, 6, 12, 13, 8)
    lm.enable()
    rm.enable()
    # FORWARD 4.5 m
    target_distance = 4.7  # m
    target_counts = target_distance / (WHEEL_RADIUS * 2 * pi) * GEAR_RATIO * CPR
    print(f"target encoder counts: {target_counts}")
    while lm.encoder_counts < target_counts and rm.encoder_counts < target_counts:
        if lm.encoder_counts - rm.encoder_counts > 32:
            lm.stop()
            rm.forward(0.5)
        elif lm.encoder_counts - rm.encoder_counts < -32:
            lm.forward(0.5)
            rm.stop()
        else:
            lm.forward(0.5)
            rm.forward(0.5)
        sleep(0.01)
    lm.stop()
    rm.stop()
    sleep(1)
    lm.reset_encoder()
    rm.reset_encoder()
    # TURN 90
    target_distance = WHEEL_SEPARATION / 2 * pi / 2  # m
    target_counts = target_distance / (WHEEL_RADIUS * 2 * pi) * GEAR_RATIO * CPR - 256
    print(f"target encoder counts: {target_counts}")
    while lm.encoder_counts < target_counts and rm.encoder_counts < target_counts:
        if lm.encoder_counts - rm.encoder_counts > 32:
            lm.stop()
            rm.forward(0.3)
        elif lm.encoder_counts - rm.encoder_counts < -32:
            lm.backward(0.3)
            rm.stop()
        else:
            lm.backward(0.3)
            rm.forward(0.3)
        sleep(0.01)
    lm.stop()
    rm.stop()
    sleep(1)
    lm.reset_encoder()
    rm.reset_encoder()
    # FORWARD 8.5 m
    target_distance = 9  # m
    target_counts = target_distance / (WHEEL_RADIUS * 2 * pi) * GEAR_RATIO * CPR
    print(f"target encoder counts: {target_counts}")
    while lm.encoder_counts < target_counts and rm.encoder_counts < target_counts:
        if lm.encoder_counts - rm.encoder_counts > 32:
            lm.stop()
            rm.forward(0.5)
        elif lm.encoder_counts - rm.encoder_counts < -32:
            lm.forward(0.5)
            rm.stop()
        else:
            lm.forward(0.5)
            rm.forward(0.5)
        sleep(0.01)
    lm.stop()
    rm.stop()
    sleep(1)
    lm.reset_encoder()
    rm.reset_encoder()

    lm.disable()
    rm.disable()
    freq(125_000_000)

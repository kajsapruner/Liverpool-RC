from math import pi
from machine import Pin, PWM
from time import sleep

class Motor:
    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, slp_pin, ab_order=1, frequency=1000):
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._slp_pin = Pin(slp_pin, Pin.OUT)
        self._pwm_pin.freq(frequency)
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._enca_handler)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encb_handler)
        self.ab_order = ab_order
        self.CPR = 64
        self.GEAR_RATIO = 70
        self.encoder_counts = 0
        self.enca_val = self._enca_pin.value()
        self.encb_val = self._encb_pin.value()
        self.stop()

    def _enca_handler(self, pin):
        self.enca_val = pin.value()
        if self.enca_val == 1:
            if self.encb_val == 0:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1
        else:
            if self.encb_val == 1:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1

    def _encb_handler(self, pin):
        self.encb_val = pin.value()
        if self.encb_val == 1:
            if self.enca_val == 0:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1
        else:
            if self.enca_val == 1:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1

    def stop(self):
        self._pwm_pin.duty_u16(0)
        self._slp_pin.value(0)

    def forward(self, duty=1.0):
        self._slp_pin.value(1)
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty * 65536))

    def backward(self, duty=1.0):
        self._slp_pin.value(1)
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty * 65536))

    def reset_encoder(self):
        self.encoder_counts = 0

class PicoController:
    def __init__(self, left_enc_a, left_enc_b, left_dir_pin, left_pwm_pin, left_slp_pin,
                 right_enc_a, right_enc_b, right_dir_pin, right_pwm_pin, right_slp_pin,
                 wheel_radius, encoder_resolution):
        self.left_motor = Motor(left_dir_pin, left_pwm_pin, left_enc_a, left_enc_b, left_slp_pin)
        self.right_motor = Motor(right_dir_pin, right_pwm_pin, right_enc_a, right_enc_b, right_slp_pin)
        self.wheel_radius = wheel_radius
        self.encoder_resolution = encoder_resolution

    def move_forward(self, speed):
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)

    def move_backward(self, speed):
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)

    def turn_left(self, speed):
        self.left_motor.backward(speed)
        self.right_motor.forward(speed)

    def turn_right(self, speed):
        self.left_motor.forward(speed)
        self.right_motor.backward(speed)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

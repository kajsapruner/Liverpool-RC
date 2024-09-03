from machine import Pin, PWM
from time import sleep

class WheelDriver:
    def __init__(self, dir_pin, pwm_pin, slp_pin):
        self.DIR_PIN = Pin(dir_pin, Pin.OUT)
        self.PWM_PIN = PWM(Pin(pwm_pin))
        self.PWM_PIN.freq(1000)
        self.SLP_PIN = Pin(slp_pin, Pin.OUT)
        
        self.SLP_PIN.value(1)  # Wake up the motor driver
        
    def forward(self, speed=0.5):
        assert 0 <= speed <= 1
        self.DIR_PIN.value(1)  # Set direction to forward
        self.PWM_PIN.duty_u16(int(65535 * speed))  # Set speed

    def backward(self, speed=0.5):
        assert 0 <= speed <= 1
        self.DIR_PIN.value(0)  # Set direction to backward
        self.PWM_PIN.duty_u16(int(65535 * speed))  # Set speed
        
    def stop(self):
        self.PWM_PIN.duty_u16(0)  # Stop the motor

if __name__ == '__main__':
    # Initialize both wheels
    right_wheel = WheelDriver(dir_pin=7, pwm_pin=6, slp_pin=8)
    left_wheel = WheelDriver(dir_pin=3, pwm_pin=2, slp_pin=4)

    # Move forward
    left_wheel.forward(0.5)
    right_wheel.forward(0.5)
    sleep(2)

    # Move backward
    left_wheel.backward(0.5)
    right_wheel.backward(0.5)
    sleep(2)

    # Turn left (left wheel stops, right wheel moves forward)
    left_wheel.stop()
    right_wheel.forward(0.5)
    sleep(2)

    # Stop the robot
    left_wheel.stop()
    right_wheel.stop()
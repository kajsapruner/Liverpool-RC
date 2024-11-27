import sys
from math import pi
from machine import Pin, PWM, Timer
from time import sleep

class Motor:

    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, slp_pin, ab_order=1, frequency=1000):
        # set pins
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._slp_pin = Pin(slp_pin, Pin.OUT)  # Sleep pin
        self._pwm_pin.freq(frequency)
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._enca_handler)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encb_handler)
        # constants
        self.ab_order = ab_order  # 1: a triggers first; -1: b triggers first
        self.CPR = 64
        self.GEAR_RATIO = 70
        self.encoder_counts = 0
        self.enca_val = self._enca_pin.value()
        self.encb_val = self._encb_pin.value()

        # Stop motor initially
        self.stop()

    def _enca_handler(self, pin):
        # Handle encoder signal change
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
        # Handle encoder signal change
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
        # Stop the motor and put to sleep
        self._pwm_pin.duty_u16(0)
        self._slp_pin.value(0)  # Sleep motor

    def forward(self, duty=1.0):
        # Move motor forward
        self._slp_pin.value(1)  # Wake up motor
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty*65536))

    def backward(self, duty=1.0):
        # Move motor backward
        self._slp_pin.value(1)  # Wake up motor
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty*65536))

    def set_velocity(self, velocity):
        # For direct control, you can set desired velocity here
        self.target_velocity = velocity

    def reset_encoder(self):
        # Reset encoder counts
        self.encoder_counts = 0

class Robot:

    def __init__(self, left_motor_pins, right_motor_pins, frequency=1000):
        assert len(left_motor_pins) == 5
        assert len(right_motor_pins) == 5
        self._left_motor = Motor(
            dir_pin=left_motor_pins[0],
            pwm_pin=left_motor_pins[1],
            enca_pin=left_motor_pins[2],
            encb_pin=left_motor_pins[3],
            slp_pin=left_motor_pins[4],
            ab_order=1,
            frequency=frequency
        )
        self._right_motor = Motor(
            dir_pin=right_motor_pins[0],
            pwm_pin=right_motor_pins[1],
            enca_pin=right_motor_pins[2],
            encb_pin=right_motor_pins[3],
            slp_pin=right_motor_pins[4],
            ab_order=-1,
            frequency=frequency
        )

        self.WHEEL_RADIUS = 0.05
        self.WHEEL_SEPARATION = 0.254

    def forward(self, speed=1.0):
        self._left_motor.forward(duty=speed)
        self._right_motor.forward(duty=speed)

    def backward(self, speed=1.0):
        self._left_motor.backward(duty=speed)
        self._right_motor.backward(duty=speed)

    def stop(self):
        self._left_motor.stop()
        self._right_motor.stop()

    def set_velocity(self, target_lin, target_ang):
        # Set velocity based on desired linear and angular velocity
        left_target_vel = (target_lin - (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS
        right_target_vel = (target_lin + (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS
        self._left_motor.set_velocity(left_target_vel)
        self._right_motor.set_velocity(right_target_vel)

def move_forward_distance(robot, distance, speed=0.5):
    # Calculate the total encoder counts needed to cover the distance
    wheel_circumference = 2 * pi * robot.WHEEL_RADIUS
    wheel_rotations = distance / wheel_circumference
    target_counts = int((wheel_rotations * robot._left_motor.GEAR_RATIO) * (robot._left_motor.CPR))

    # Reset encoder counts
    robot._left_motor.reset_encoder()
    robot._right_motor.reset_encoder()

    # Start moving forward
    robot.forward(speed)

    while robot._left_motor.encoder_counts <= target_counts and robot._right_motor.encoder_counts <= target_counts:
        # Get current encoder counts
        print(f"Left Encoder: {robot._left_motor.encoder_counts}, Right Encoder: {robot._right_motor.encoder_counts}")
        
        if (abs(robot._left_motor.encoder_counts) > abs(robot._right_motor.encoder_counts) + 20):
            robot._left_motor.stop()
            robot._right_motor.forward(speed)
        elif (abs(robot._right_motor.encoder_counts) > abs(robot._left_motor.encoder_counts) + 20):
            robot._right_motor.stop()
            robot._left_motor.forward(speed)
        else:
            robot._right_motor.forward(speed)
            robot._left_motor.forward(speed)
        
        # Check if target counts are reached
        if max(abs(robot._left_motor.encoder_counts), abs(robot._right_motor.encoder_counts)) >= target_counts:
            print(f"Moved {distance} meters.")
            break
        
        sleep(0.01)

    # Stop the robot
    robot.stop()


def turn_90_degrees(robot, clockwise=True, speed=0.5):
    # Calculate the arc distance each wheel needs to travel
    arc_distance = (pi * robot.WHEEL_SEPARATION) / 8  

    # Calculate target encoder counts for the arc distance
    target_counts = (arc_distance / (2 * pi * robot.WHEEL_RADIUS)) * robot._left_motor.CPR * robot._left_motor.GEAR_RATIO

    # Reset encoder counts
    robot._left_motor.reset_encoder()
    robot._right_motor.reset_encoder()

    # Start turning
    if clockwise:
        robot._left_motor.forward(speed)  # Left motor forward
        robot._right_motor.backward(speed)  # Right motor backward
    else:
        robot._left_motor.backward(speed)  # Left motor backward
        robot._right_motor.forward(speed)  # Right motor forward

    while True:
        # Get current encoder counts
        left_counts = abs(robot._left_motor.encoder_counts)
        right_counts = abs(robot._right_motor.encoder_counts)

        # Check if target counts are reached
        if max(left_counts, right_counts) >= target_counts:
            print("90-degree turn complete.")
            break

    # Stop the robot
    robot.stop()


if __name__ == "__main__":
    # Initialize your robot instance
    robot = Robot(left_motor_pins=[3, 2, 14, 15, 4], right_motor_pins=[7, 6, 12, 13, 8])

    # Move forward 2 meters
    print("Moving forward 7 meters...")
    #move_forward_distance(robot, distance=1, speed=0.5)
    #robot.stop()
    #sleep(5)
    # Turn 90 degrees clockwise
    print("Turning 90 degrees clockwise...")
    turn_90_degrees(robot, clockwise=True)

    print("Finished.")


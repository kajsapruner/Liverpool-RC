from machine import Pin, PWM
from time import sleep

class WheelDriver:
    def __init__(self, enca_pin_id, encb_pin_id, dir_pin, pwm_pin, slp_pin, wheel_radius, encoder_resolution):
        self.DIR_PIN = Pin(dir_pin, Pin.OUT)
        self.PWM_PIN = PWM(Pin(pwm_pin))
        self.PWM_PIN.freq(1000)
        self.SLP_PIN = Pin(slp_pin, Pin.OUT)
        self.SLP_PIN.value(1)  # Wake up the motor driver

        self.wheel_radius = wheel_radius  # in meters
        self.encoder_resolution = encoder_resolution  # counts per revolution
        
        # Initialize encoder pins
        self.enca = Pin(enca_pin_id, Pin.IN)
        self.encb = Pin(encb_pin_id, Pin.IN)
        self.encoder_count = 0
        
        # Setup encoder reading (simplified for this example)
        self.enca.irq(trigger=Pin.IRQ_RISING, handler=self.update_encoder)

    def update_encoder(self, pin):
        # Simple encoder count update
        if self.encb.value() == 0:
            self.encoder_count += 1
        else:
            self.encoder_count -= 1
    
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

    def calculate_encoder_counts(self, distance):
        # Calculate the circumference of the wheel
        wheel_circumference = 2 * 3.14159 * self.wheel_radius  # in meters
        
        # Calculate the number of wheel rotations needed to cover the distance
        rotations_needed = distance / wheel_circumference
        
        # Calculate the number of encoder counts needed
        encoder_counts_needed = int(rotations_needed * self.encoder_resolution)
        
        return encoder_counts_needed

class Virgil:
    def __init__(self, left_wheel_driver, right_wheel_driver):
        self.left_wheel = left_wheel_driver
        self.right_wheel = right_wheel_driver

    def forward(self, distance, speed=0.5):
        # Move both wheels forward
        target_counts = self.left_wheel.calculate_encoder_counts(distance)  # Calculate target counts for both wheels
        self.left_wheel.encoder_count = 0  # Reset encoder counts for both wheels
        self.right_wheel.encoder_count = 0  # Reset encoder counts for both wheels

        self.left_wheel.forward(speed)  # Start moving both wheels forward
        self.right_wheel.forward(speed)

        while abs(self.left_wheel.encoder_count) < target_counts:
            sleep(0.01)  # Short delay to avoid hogging the CPU
            
        self.left_wheel.stop()  # Stop both wheels after reaching the target distance
        self.right_wheel.stop()
        
    def right(self, distance, speed=0.5):
        # Move both wheels forward
        target_counts = self.left_wheel.calculate_encoder_counts(distance)  # Calculate target counts for both wheels
        self.left_wheel.encoder_count = 0  # Reset encoder counts for both wheels
        self.right_wheel.encoder_count = 0  # Reset encoder counts for both wheels

        self.left_wheel.forward(speed)  # Start moving both wheels forward

        while abs(self.left_wheel.encoder_count) < target_counts:
            sleep(0.01)  # Short delay to avoid hogging the CPU
            
        self.left_wheel.stop()  # Stop both wheels after reaching the target distance
        self.right_wheel.stop()
        
    def left(self, distance, speed=0.5):
        # Move both wheels forward
        target_counts = self.left_wheel.calculate_encoder_counts(distance)  # Calculate target counts for both wheels
        self.left_wheel.encoder_count = 0  # Reset encoder counts for both wheels
        self.right_wheel.encoder_count = 0  # Reset encoder counts for both wheels

        self.right_wheel.forward(speed)  # Start moving both wheels forward

        while abs(self.right_wheel.encoder_count) < target_counts:
            sleep(0.01)  # Short delay to avoid hogging the CPU
            
        self.left_wheel.stop()  # Stop both wheels after reaching the target distance
        self.right_wheel.stop()

if __name__ == '__main__':
    # Example encoder resolution: 400 counts per revolution
    right_wheel = WheelDriver(enca_pin_id=12, encb_pin_id=13, dir_pin=7, pwm_pin=6, slp_pin=8, wheel_radius=0.06, encoder_resolution=400)
    left_wheel = WheelDriver(enca_pin_id=14, encb_pin_id=15, dir_pin=3, pwm_pin=2, slp_pin=4, wheel_radius=0.06, encoder_resolution=400)
    
    virgil = Virgil(left_wheel, right_wheel) 
    virgil.forward(29, speed=.5)
    virgil.left(1.3, speed=.5)
    virgil.forward(28, speed=.5)
    virgil.left(1.3, speed=.5)
    virgil.forward(11.75, speed=.5)
    virgil.left(1.3, speed=.5)
    virgil.forward(24, speed=.5)
    virgil.right(1.3, speed=.5)
    virgil.forward(10.5, speed=.5)
    virgil.right(1.3, speed=.5)
    virgil.forward(24, speed=.5)
    virgil.left(1.3, speed=.5)
    virgil.forward(11.75, speed=.5)
    virgil.left(1.3, speed=.5)
    virgil.forward(30, speed=.5)
    
    left_wheel.stop()
    right_wheel.stop() 



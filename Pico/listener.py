from autonomous_controller import PicoController
from time import sleep_ms

# SETUP - Initialize the Pico controller with the appropriate pin configuration
virgil = PicoController(
    left_enc_a=14, left_enc_b=15, left_dir_pin=3, left_pwm_pin=2, left_slp_pin=4,
    right_enc_a=12, right_enc_b=13, right_dir_pin=7, right_pwm_pin=6, right_slp_pin=8,
    wheel_radius=0.06, encoder_resolution=400
)

# LOOP - Listen for commands
while True:
    buffer = input().strip()  # Blocking call for user input
    if buffer:
        print(f"Received command: {buffer}")
        # Parse and act on the command
        try:
            cmd, value = buffer.split(',')
            value = int(value)
            if cmd == "FORWARD":
                virgil.move_forward(value / 100)  # Scale speed to percentage
            elif cmd == "BACKWARD":
                virgil.move_backward(value / 100)
            elif cmd == "LEFT":
                virgil.turn_left(value / 100)
            elif cmd == "RIGHT":
                virgil.turn_right(value / 100)
            elif cmd == "STOP":
                virgil.stop()
            else:
                print("Unknown command!")
        except ValueError:
            print("Invalid command format! Correct format: COMMAND,value (e.g., FORWARD,50)")

    # Optional delay to prevent high CPU usage
    sleep_ms(10)

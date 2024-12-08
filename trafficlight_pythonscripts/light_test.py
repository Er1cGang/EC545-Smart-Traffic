import RPi.GPIO as GPIO
import time

# GPIO pins for LEDs
led_pins = [14, 15, 18, 16, 20, 21]

# Setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(led_pins, GPIO.OUT)  # Set pins as output

def blink_leds(delay=0.5, iterations=5):
    """
    Blink LEDs on the specified GPIO pins.
    :param delay: Time delay between on and off states (in seconds).
    :param iterations: Number of blink cycles.
    """
    for _ in range(iterations):
        GPIO.output(led_pins, GPIO.HIGH)  # Turn all LEDs on
        time.sleep(delay)
        GPIO.output(led_pins, GPIO.LOW)  # Turn all LEDs off
        time.sleep(delay)

try:
    print("Blinking LEDs...")
    blink_leds(delay=0.5, iterations=10)  # Adjust delay/iterations as needed
finally:
    GPIO.cleanup()  # Reset GPIO pins to default state
    print("Cleaned up GPIO and exiting.")

import time
import smbus
import board
import digitalio
import adafruit_vcnl4010
import adafruit_apds9960.apds9960
import RPi.GPIO as GPIO

# I2C bus
bus = smbus.SMBus(1)
# TCA9548A I2C address (default: 0x70)
tca_address = 0x70

# GPIO Pins for the LEDs on the first side (VCNL4010 sensors)
GREEN_LED_PIN = 18  # GPIO pin connected to the green LED (Side 1)
YELLOW_LED_PIN = 15  # GPIO pin connected to the yellow LED (Side 1)
RED_LED_PIN = 14  # GPIO pin connected to the red LED (Side 1)

# GPIO Pins for the LEDs on the second side (APDS-9960 sensors)
SECOND_GREEN_LED_PIN = 21  # GPIO pin connected to the green LED (Side 2)
SECOND_YELLOW_LED_PIN = 20  # GPIO pin connected to the yellow LED (Side 2)
SECOND_RED_LED_PIN = 16  # GPIO pin connected to the red LED (Side 2)

# Proximity thresholds for different sensors
VCNL4010_PROXIMITY_THRESHOLD = 4000  # Keep original VCNL4010 threshold
APDS9960_PROXIMITY_THRESHOLD = 50  # New threshold for APDS-9960

# Timing constants
NO_CAR_WAIT_TIME = 5  # Wait 5 seconds if no cars detected
MAX_GREEN_LIGHT_TIME = 15  # Maximum green light duration

# Function to select a channel on the TCA9548A
def select_tca_channel(channel):
    # Enable the channel by writing to TCA9548A
    bus.write_byte(tca_address, 1 << channel)
    time.sleep(0.1)

# Setup GPIO for both sides
GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(YELLOW_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(SECOND_GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(SECOND_YELLOW_LED_PIN, GPIO.OUT)
GPIO.setup(SECOND_RED_LED_PIN, GPIO.OUT)

# Set up I2C for the VCNL4010 and APDS-9960 sensors
i2c = board.I2C()

# Initialize the VCNL4010 sensors
select_tca_channel(0)  # Channel 0 for first VCNL4010
vcnl_sensor1 = adafruit_vcnl4010.VCNL4010(i2c)
select_tca_channel(1)  # Channel 1 for second VCNL4010
vcnl_sensor2 = adafruit_vcnl4010.VCNL4010(i2c)

# Initialize the APDS-9960 sensors
select_tca_channel(2)  # Channel 2 for first APDS-9960
apds_sensor1 = adafruit_apds9960.apds9960.APDS9960(i2c)
apds_sensor1.enable_proximity = True
select_tca_channel(3)  # Channel 3 for second APDS-9960
apds_sensor2 = adafruit_apds9960.apds9960.APDS9960(i2c)
apds_sensor2.enable_proximity = True

# Helper functions to turn off LEDs
def turn_off_all_leds_side_1():
    GPIO.output(GREEN_LED_PIN, GPIO.LOW)
    GPIO.output(YELLOW_LED_PIN, GPIO.LOW)
    GPIO.output(RED_LED_PIN, GPIO.LOW)

def turn_off_all_leds_side_2():
    GPIO.output(SECOND_GREEN_LED_PIN, GPIO.LOW)
    GPIO.output(SECOND_YELLOW_LED_PIN, GPIO.LOW)
    GPIO.output(SECOND_RED_LED_PIN, GPIO.LOW)

# Initial LED setup
turn_off_all_leds_side_1()
turn_off_all_leds_side_2()
GPIO.output(GREEN_LED_PIN, GPIO.HIGH)  # Side 1 Green ON
GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)  # Side 2 Red ON

# Flags and timers
side_1_is_green = True
proximity_triggered = False
last_car_detection_time = time.time()
green_light_start_time = time.time()

try:
    print("Enhanced Traffic Light Control with Timing Logic")
    
    while True:
        # Read both VCNL4010 proximities (Side 1)
        select_tca_channel(0)  # Channel 0 for first VCNL sensor
        vcnl_proximity1 = vcnl_sensor1.proximity
        select_tca_channel(1)  # Channel 1 for second VCNL sensor
        vcnl_proximity2 = vcnl_sensor2.proximity

        # Read both APDS-9960 proximities (Side 2)
        select_tca_channel(2)  # Channel 2 for first APDS sensor
        apds_proximity1 = apds_sensor1.proximity
        select_tca_channel(3)  # Channel 3 for second APDS sensor
        apds_proximity2 = apds_sensor2.proximity

        # Current time for tracking
        current_time = time.time()
        
        # Logic for Side 1 (VCNL4010)
        if side_1_is_green:
            # Check if cars are detected on Side 1
            if (vcnl_proximity1 > VCNL4010_PROXIMITY_THRESHOLD) or (vcnl_proximity2 > VCNL4010_PROXIMITY_THRESHOLD):
                last_car_detection_time = current_time
            
            # Check conditions for switching lights
            time_since_last_car = current_time - last_car_detection_time
            time_green_light = current_time - green_light_start_time
            
            # Switch lights if:
            # 1. No cars detected for 5 seconds, OR
            # 2. Green light has been on for more than 15 seconds
            if (time_since_last_car >= NO_CAR_WAIT_TIME) or (time_green_light >= MAX_GREEN_LIGHT_TIME):
                # Cycle Side 1 lights
                turn_off_all_leds_side_1()
                GPIO.output(YELLOW_LED_PIN, GPIO.HIGH)
                print("Side 1 - Yellow LED is ON")
                time.sleep(2)  # Wait for 2 seconds

                # Side 1: Turn Red
                turn_off_all_leds_side_1()
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                print("Side 1 - Red LED is ON")

                # Both sides red for 2 seconds
                GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)
                print("Both sides - Red LEDs are ON")
                time.sleep(2)

                # Side 2: Turn Green
                turn_off_all_leds_side_1()
                turn_off_all_leds_side_2()
                GPIO.output(SECOND_GREEN_LED_PIN, GPIO.HIGH)
                GPIO.output(RED_LED_PIN, GPIO.HIGH)  # Keep Side 1 Red
                print("Side 2 - Green LED is ON, Side 1 - Red LED is ON")
                
                # Reset timers and flags
                side_1_is_green = False
                last_car_detection_time = current_time
                green_light_start_time = current_time

        # Logic for Side 2 (APDS-9960)
        else:
            # Check if cars are detected on Side 2
            if (apds_proximity1 > APDS9960_PROXIMITY_THRESHOLD) or (apds_proximity2 > APDS9960_PROXIMITY_THRESHOLD):
                last_car_detection_time = current_time
            
            # Check conditions for switching lights
            time_since_last_car = current_time - last_car_detection_time
            time_green_light = current_time - green_light_start_time
            
            # Switch lights if:
            # 1. No cars detected for 5 seconds, OR
            # 2. Green light has been on for more than 15 seconds
            if (time_since_last_car >= NO_CAR_WAIT_TIME) or (time_green_light >= MAX_GREEN_LIGHT_TIME):
                # Cycle Side 2 lights
                turn_off_all_leds_side_2()
                GPIO.output(SECOND_YELLOW_LED_PIN, GPIO.HIGH)
                print("Side 2 - Yellow LED is ON")
                time.sleep(2)  # Wait for 2 seconds

                # Side 2: Turn Red
                turn_off_all_leds_side_2()
                GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)
                print("Side 2 - Red LED is ON")

                # Both sides red for 2 seconds
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                print("Both sides - Red LEDs are ON")
                time.sleep(2)

                # Side 1: Turn Green
                turn_off_all_leds_side_1()
                turn_off_all_leds_side_2()
                GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
                GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)  # Keep Side 2 Red
                print("Side 1 - Green LED is ON, Side 2 - Red LED is ON")
                
                # Reset timers and flags
                side_1_is_green = True
                last_car_detection_time = current_time
                green_light_start_time = current_time

        time.sleep(0.1)  # Small delay for stability

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    turn_off_all_leds_side_1()  # Ensure all LEDs are off on Side 1
    turn_off_all_leds_side_2()  # Ensure all LEDs are off on Side 2
    GPIO.cleanup()  # Reset GPIO settings
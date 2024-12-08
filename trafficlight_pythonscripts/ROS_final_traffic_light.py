#!/usr/bin/python3
# Activate virtual environment
# python3 -m venv myenv
# source myenv/bin/activate

# pip install adafruit-circuitpython-vcnl4010
# pip install adafruit-circuitpython-apds9960
# pip install adafruit-circuitpython-tca9548a

import time
import smbus
import board
import digitalio
import adafruit_vcnl4010
import adafruit_apds9960.apds9960
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import String
# from traffic_controller.msg import TrafficSignal

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

# Debounce time in seconds
DEBOUNCE_TIME = 0.5

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

# Flag to track which side's light is currently active
side_1_is_green = True

# Debounce tracking variables
side_1_trigger_start_time = 0
side_2_trigger_start_time = 0
side_1_currently_triggered = False
side_2_currently_triggered = False

try:
    print("Starting Traffic Light Control")
    rospy.init_node("rospy_light_signal", anonymous=False)
    pub = rospy.Publisher('/rpi_light_signal', String, queue_size=10)
    message = "FF"
    pub.publish(message)
    
    while True:
        # Read both VCNL4010 proximities (Side 1)
        select_tca_channel(0)  # Channel 0 for first VCNL sensor
        vcnl_proximity1 = vcnl_sensor1.proximity
        select_tca_channel(1)  # Channel 1 for second VCNL sensor
        vcnl_proximity2 = vcnl_sensor2.proximity
        print(f"VCNL4010 Sensor 1 Proximity: {vcnl_proximity1}")
        print(f"VCNL4010 Sensor 2 Proximity: {vcnl_proximity2}")

        # Read both APDS-9960 proximities (Side 2)
        select_tca_channel(2)  # Channel 2 for first APDS sensor
        apds_proximity1 = apds_sensor1.proximity
        select_tca_channel(3)  # Channel 3 for second APDS sensor
        apds_proximity2 = apds_sensor2.proximity
        print(f"APDS-9960 Sensor 1 Proximity: {apds_proximity1}")
        print(f"APDS-9960 Sensor 2 Proximity: {apds_proximity2}")

        current_time = time.time()

        # Check if objects are near on Side 1 (VCNL4010)
        if (vcnl_proximity1 > VCNL4010_PROXIMITY_THRESHOLD) or (vcnl_proximity2 > VCNL4010_PROXIMITY_THRESHOLD):
            if not side_1_currently_triggered:
                side_1_trigger_start_time = current_time
                side_1_currently_triggered = True
        else:
            side_1_currently_triggered = False
            side_1_trigger_start_time = 0

        # Check if objects are near on Side 2 (APDS-9960)
        if (apds_proximity1 > APDS9960_PROXIMITY_THRESHOLD) or (apds_proximity2 > APDS9960_PROXIMITY_THRESHOLD):
            if not side_2_currently_triggered:
                side_2_trigger_start_time = current_time
                side_2_currently_triggered = True
        else:
            side_2_currently_triggered = False
            side_2_trigger_start_time = 0

        # Trigger logic for Side 1
        if side_1_currently_triggered and (current_time - side_1_trigger_start_time >= DEBOUNCE_TIME):
            if side_1_is_green:
                print("Object detected on Side 1! Cycling LEDs...")
                message = "FF"
                pub.publish(message)

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
                message = "TF"
                pub.publish(message)

                turn_off_all_leds_side_1()
                turn_off_all_leds_side_2()
                GPIO.output(SECOND_GREEN_LED_PIN, GPIO.HIGH)
                GPIO.output(RED_LED_PIN, GPIO.HIGH)  # Keep Side 1 Red
                print("Side 2 - Green LED is ON, Side 1 - Red LED is ON")
                
                # Set the flag to indicate that side 2's light is now active
                side_1_is_green = False
        
        # Trigger logic for Side 2
        elif side_2_currently_triggered and (current_time - side_2_trigger_start_time >= DEBOUNCE_TIME):
            if not side_1_is_green:
                print("Object detected on Side 2! Cycling LEDs...")
                message = "FF"
                pub.publish(message)

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
                message = "FT"
                pub.publish(message)

                turn_off_all_leds_side_1()
                turn_off_all_leds_side_2()
                GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
                GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)  # Keep Side 2 Red
                print("Side 1 - Green LED is ON, Side 2 - Red LED is ON")
                
                # Set the flag to indicate that side 1's light is now active
                side_1_is_green = True

        time.sleep(0.1)  # Small delay for stability

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    turn_off_all_leds_side_1()  # Ensure all LEDs are off on Side 1
    turn_off_all_leds_side_2()  # Ensure all LEDs are off on Side 2
    GPIO.cleanup()  # Reset GPIO settings
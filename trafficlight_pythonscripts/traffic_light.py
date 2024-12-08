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
VCNL4010_PROXIMITY_THRESHOLD = 4000
APDS9960_PROXIMITY_THRESHOLD = 50

# Traffic light control parameters
MANDATORY_GREEN_DURATION = 15  # Mandatory green light duration in seconds
CAR_GROUP_WINDOW = 5  # Window to allow a group of cars to pass after first detection

class TrafficLightController:
    def __init__(self):
        print("Initializing Traffic Light Controller...")
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
        GPIO.setup(YELLOW_LED_PIN, GPIO.OUT)
        GPIO.setup(RED_LED_PIN, GPIO.OUT)
        GPIO.setup(SECOND_GREEN_LED_PIN, GPIO.OUT)
        GPIO.setup(SECOND_YELLOW_LED_PIN, GPIO.OUT)
        GPIO.setup(SECOND_RED_LED_PIN, GPIO.OUT)

        # Initialize I2C and sensor channels
        self.i2c = board.I2C()
        self.setup_sensors()

        # Traffic light state variables
        self.side_1_is_green = True
        self.green_light_start_time = time.time()
        
        # Car detection variables
        self.side_1_car_timestamps = []
        self.side_2_car_timestamps = []
        
        # Last car detection times for group window
        self.side_1_last_car_time = 0
        self.side_2_last_car_time = 0

        # Initial LED setup
        self.turn_off_all_leds()
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)  # Side 1 Green ON
        GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)  # Side 2 Red ON
        print("Initial state: Side 1 GREEN, Side 2 RED")

    def setup_sensors(self):
        print("Setting up sensors...")
        def select_tca_channel(channel):
            bus.write_byte(tca_address, 1 << channel)
            time.sleep(0.1)

        # VCNL4010 Sensors
        select_tca_channel(0)
        self.vcnl_sensor1 = adafruit_vcnl4010.VCNL4010(self.i2c)
        select_tca_channel(1)
        self.vcnl_sensor2 = adafruit_vcnl4010.VCNL4010(self.i2c)

        # APDS-9960 Sensors
        select_tca_channel(2)
        self.apds_sensor1 = adafruit_apds9960.apds9960.APDS9960(self.i2c)
        self.apds_sensor1.enable_proximity = True
        select_tca_channel(3)
        self.apds_sensor2 = adafruit_apds9960.apds9960.APDS9960(self.i2c)
        self.apds_sensor2.enable_proximity = True
        print("Sensors setup complete.")

    def turn_off_all_leds(self):
        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        GPIO.output(YELLOW_LED_PIN, GPIO.LOW)
        GPIO.output(RED_LED_PIN, GPIO.LOW)
        GPIO.output(SECOND_GREEN_LED_PIN, GPIO.LOW)
        GPIO.output(SECOND_YELLOW_LED_PIN, GPIO.LOW)
        GPIO.output(SECOND_RED_LED_PIN, GPIO.LOW)
        
    def read_vcnl_sensors(self):
        def select_tca_channel(channel):
            bus.write_byte(tca_address, 1 << channel)
            time.sleep(0.1)

        select_tca_channel(0)
        vcnl_proximity1 = self.vcnl_sensor1.proximity
        select_tca_channel(1)
        vcnl_proximity2 = self.vcnl_sensor2.proximity

        return vcnl_proximity1, vcnl_proximity2

    def read_apds_sensors(self):
        def select_tca_channel(channel):
            bus.write_byte(tca_address, 1 << channel)
            time.sleep(0.1)

        select_tca_channel(2)
        apds_proximity1 = self.apds_sensor1.proximity
        select_tca_channel(3)
        apds_proximity2 = self.apds_sensor2.proximity

        return apds_proximity1, apds_proximity2

    def switch_traffic_light(self):
        print("\n--- SWITCHING TRAFFIC LIGHT ---")
        # Turn yellow first
        if self.side_1_is_green:
            print("Side 1: Changing from GREEN to YELLOW")
            print(f"Side 1 had {len(self.side_1_car_timestamps)} cars detected during green cycle")
            self.turn_off_all_leds()
            GPIO.output(YELLOW_LED_PIN, GPIO.HIGH)
            GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)
        else:
            print("Side 2: Changing from GREEN to YELLOW")
            print(f"Side 2 had {len(self.side_2_car_timestamps)} cars detected during green cycle")
            self.turn_off_all_leds()
            GPIO.output(SECOND_YELLOW_LED_PIN, GPIO.HIGH)
            GPIO.output(RED_LED_PIN, GPIO.HIGH)

        time.sleep(2)  # Yellow light duration
        print("Yellow light duration complete")

        # Turn red
        self.turn_off_all_leds()
        if self.side_1_is_green:
            print("Side 1: Changing from YELLOW to RED")
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
        else:
            print("Side 2: Changing from YELLOW to RED")
            GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)

        # Both sides red for 2 seconds
        if self.side_1_is_green:
            GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)
        else:
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
        print("Both sides RED for 2 seconds")
        time.sleep(2)

        # Switch to the other side's green light
        self.turn_off_all_leds()
        if self.side_1_is_green:
            print("Switching: Side 2 GREEN, Side 1 RED")
            GPIO.output(SECOND_GREEN_LED_PIN, GPIO.HIGH)
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
        else:
            print("Switching: Side 1 GREEN, Side 2 RED")
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            GPIO.output(SECOND_RED_LED_PIN, GPIO.HIGH)

        # Reset state and time
        self.side_1_is_green = not self.side_1_is_green
        self.green_light_start_time = time.time()
        self.side_1_car_timestamps.clear()
        self.side_2_car_timestamps.clear()
        self.side_1_last_car_time = 0
        self.side_2_last_car_time = 0
        print("--- TRAFFIC LIGHT SWITCH COMPLETE ---\n")
        
    def run(self):
        try:
            print("Starting Traffic Light Control")
            
            while True:
                current_time = time.time()
                
                # Read sensor proximities
                vcnl_proximity1, vcnl_proximity2 = self.read_vcnl_sensors()
                apds_proximity1, apds_proximity2 = self.read_apds_sensors()

                # Detailed sensor reading print
                print(f"\n--- Sensor Readings at {current_time:.2f} seconds ---")
                print(f"Side 1 VCNL Sensors: {vcnl_proximity1}, {vcnl_proximity2}")
                print(f"Side 2 APDS Sensors: {apds_proximity1}, {apds_proximity2}")

                # Check cars on Side 1 (VCNL4010)
                side_1_car_detected = (vcnl_proximity1 > VCNL4010_PROXIMITY_THRESHOLD) or \
                                      (vcnl_proximity2 > VCNL4010_PROXIMITY_THRESHOLD)
                
                # Check cars on Side 2 (APDS-9960)
                side_2_car_detected = (apds_proximity1 > APDS9960_PROXIMITY_THRESHOLD) or \
                                      (apds_proximity2 > APDS9960_PROXIMITY_THRESHOLD)

                # Record car timestamps and update last car times
                if side_1_car_detected and self.side_1_is_green:
                    self.side_1_car_timestamps.append(current_time)
                    self.side_1_last_car_time = current_time
                    print(f"🚗 Car detected on Side 1 (Green Light) at {current_time:.2f} seconds")
                
                if side_2_car_detected and not self.side_1_is_green:
                    self.side_2_car_timestamps.append(current_time)
                    self.side_2_last_car_time = current_time
                    print(f"🚗 Car detected on Side 2 (Green Light) at {current_time:.2f} seconds")

                # Conditions to switch traffic light
                green_duration = current_time - self.green_light_start_time
                
                # Print current state information
                print(f"\n--- Current Traffic Light State ---")
                print(f"Current Side: {'Side 1' if self.side_1_is_green else 'Side 2'} GREEN")
                print(f"Green Light Duration: {green_duration:.2f} seconds")
                print(f"Cars on Current Side: {len(self.side_1_car_timestamps) if self.side_1_is_green else len(self.side_2_car_timestamps)}")
                
                if self.side_1_is_green:
                    # Check if group window has elapsed since last car
                    group_window_elapsed = (current_time - self.side_1_last_car_time) > CAR_GROUP_WINDOW
                    
                    if green_duration > MANDATORY_GREEN_DURATION:
                        # Switch conditions:
                        # 1. Side 2 has cars AND current side's group window has elapsed
                        # 2. Green duration exceeded 15 seconds AND both sides have cars
                        print("\n--- Checking Switch Conditions for Side 1 ---")
                        print(f"Green Duration Condition: {green_duration:.2f} > {MANDATORY_GREEN_DURATION}")
                        print(f"Side 2 Car Detected: {side_2_car_detected}")
                        print(f"Group Window Elapsed: {group_window_elapsed}")
                        
                        if (side_2_car_detected and group_window_elapsed) or \
                           (green_duration > MANDATORY_GREEN_DURATION and 
                            side_1_car_detected and side_2_car_detected):
                            print(f"🚦 SWITCHING: Green duration {green_duration:.2f}, Conditions met")
                            self.switch_traffic_light()

                else:  # Side 2 is green
                    # Check if group window has elapsed since last car
                    group_window_elapsed = (current_time - self.side_2_last_car_time) > CAR_GROUP_WINDOW
                    
                    if green_duration > MANDATORY_GREEN_DURATION:
                        # Same switching logic as above, but for side 2
                        print("\n--- Checking Switch Conditions for Side 2 ---")
                        print(f"Green Duration Condition: {green_duration:.2f} > {MANDATORY_GREEN_DURATION}")
                        print(f"Side 1 Car Detected: {side_1_car_detected}")
                        print(f"Group Window Elapsed: {group_window_elapsed}")
                        
                        if (side_1_car_detected and group_window_elapsed) or \
                           (green_duration > MANDATORY_GREEN_DURATION and 
                            side_1_car_detected and side_2_car_detected):
                            print(f"🚦 SWITCHING: Green duration {green_duration:.2f}, Conditions met")
                            self.switch_traffic_light()

                time.sleep(0.1)  # Small delay for stability

        except KeyboardInterrupt:
            print("Exiting program...")

        finally:
            self.turn_off_all_leds()
            GPIO.cleanup()
            
def main():
    controller = TrafficLightController()
    controller.run()

if __name__ == "__main__":
    main()
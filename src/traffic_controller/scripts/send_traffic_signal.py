#!/usr/bin/python3
import time
import rospy
# from traffic_controller.msg import TrafficSignal
from std_msgs.msg import String

def main():
    rospy.init_node("send_traffic_signal", anonymous=True)
    pub = rospy.Publisher('/rpi_light_signal', String, queue_size=10) #topic name = CAV_Data
    
    green_NS = False
    green_EW = True
    last_green = 'EW'
    message = "FF"
    pub.publish(message)
    
    while True:
        input_txt = input('enter anything to switch lights: ')
        if input_txt == 'exit':
            break

        elif input_txt == 'allgreen':
            message = "TT"

        elif input_txt != '':
            if green_EW:
                green_EW = False
            elif green_NS:
                green_NS = False
            
            elif not green_NS and not green_EW:
                if last_green == 'EW':
                    green_NS = True
                    last_green = 'NS'
                elif last_green == 'NS':
                    green_EW = True
                    last_green = 'EW'

            message = ""
            if green_NS:
                message += "T"
            else:
                message += "F"
            if green_EW:
                message += "T"
            else:
                message += "F"

        pub.publish(message)
        print(f"Current Status: green_NS: {green_NS}, green_EW: {green_EW}")
        time.sleep(0.1)

if __name__ == '__main__':
    main()
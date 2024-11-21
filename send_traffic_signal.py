#!/usr/bin/python3
import time
import rospy
from traffic_controller.msg import TrafficSignal

def main():
    rospy.init_node("send_traffic_signal", anonymous=True)
    pub = rospy.Publisher('/traffic_signal',TrafficSignal,queue_size=10) #topic name = CAV_Data
    
    green_NS = False
    green_EW = True
    message = TrafficSignal()
    message.green_NS = green_NS
    message.green_EW = green_EW
    pub.publish(message)
    
    while True:
        input_txt = input('enter anything to switch lights: ')
        if input_txt != '':
            green_NS = not green_NS
            green_EW = not green_EW
        message.green_NS = green_NS
        message.green_EW = green_EW
        pub.publish(message)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
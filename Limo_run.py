#!/usr/bin/env python3

import time
from pylimo import limo
from ackermann_msgs.msg import AckermannDrive
import rospy
from LidarX2 import LidarX2

class listener:
    def __init__(self):
        self.data = None
        self.xprev = 0
        self.yprev = 0

    def callback(self, data):
        self.data = data

    def listener(self):
        rospy.init_node("vs_listener", anonymous=True)
        rospy.Subscriber("vel_steer_limo813", AckermannDrive, self.callback)

class limo_controller:
    def __init__(self):
        # Initialize LIMO robot
        self.limo = limo.LIMO()
        self.limo.EnableCommand()

        self.listener_ins = listener()

        # Initialize LiDAR
        self.lidar = LidarX2('/dev/ttyUSB0')
        if self.lidar.open():
            print("LIDAR connection opened successfully.")
        else:
            print("Failed to open LIDAR connection.")
            exit()

    def run(self):
        iter = 0
        linear_vel = 0  # Initialize with default value
        steering_angle = 0  # Initialize with default value
        self.listener_ins.listener()
        while True:
            # self.listener_ins.listener()
            measures = self.lidar.getMeasures()
            for measure in measures:
                print(f"Angle: {measure.angle}, Distance: {measure.distance}")

            print(time.time())
            if self.listener_ins.data is not None:
                # Set motion commands from ROS topic
                linear_vel = self.listener_ins.data.speed
                steering_angle = self.listener_ins.data.steering_angle
                # self.limo.SetMotionCommand(linear_vel=linear_vel, steering_angle=steering_angle)

                print(f"Limo velocity command: {linear_vel}, Limo steering: {steering_angle}")
            else:
                print('nothing received')
            time.sleep(0.1)


if __name__ == "__main__":
    limo_control = limo_controller()
    limo_control.run()
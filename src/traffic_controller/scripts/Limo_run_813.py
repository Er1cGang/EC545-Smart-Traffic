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
        rospy.Subscriber("/vel_steer_limo813", AckermannDrive, self.callback)

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
        speed_factor = 0.0
        speed_change_rate = 0.5
        while True:
            print()
            print(time.time())
            # self.listener_ins.listener()
            measures = self.lidar.getMeasures()
            min_dist = 2000
            count = 0
            for measure in measures:
                if measure.angle < 195 and measure.angle > 165:
                    count += 1
                    # print(f"dist: {measure.distance}")
                    if measure.distance < min_dist and measure.distance > 10:
                        min_dist = measure.distance
            print(f"min_dist: {min_dist}")
            print(f"count: {count}")
            if count > 1:
                speed_factor = (1-speed_change_rate)*speed_factor + speed_change_rate*max(-0.2, min(1, (min_dist-400)/800))
            print(f"speed_factor: {speed_factor}")
            
            if self.listener_ins.data is not None:
                # Set motion commands from ROS topic
                linear_vel = self.listener_ins.data.speed * speed_factor
                steering_angle = self.listener_ins.data.steering_angle
                self.limo.SetMotionCommand(linear_vel=linear_vel, steering_angle=steering_angle)

                print(f"Limo velocity command: {linear_vel}, Limo steering: {steering_angle}")
            else:
                print('nothing received')
            time.sleep(0.1)


if __name__ == "__main__":
    limo_control = limo_controller()
    limo_control.run()
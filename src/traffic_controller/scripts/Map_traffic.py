#!/usr/bin/env python3
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from traffic_controller.msg import TrafficSignal

class CAV():
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_z = 0
        self.position_x = 0
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.position_ip_z = 0
        self.position_ip_x = 0
        self.ip_velocity = 0
        self.ip_acceleration = 0
        self.kp=0
        self.ki=0
        self.kd = 0
        self.green_NS = False
        self.green_EW = False

        #construct node, subscribe and publish to corrsponding rostopics
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+self.node_name,AckermannDrive,queue_size=10) #topic name = CAV_Data
        self.traf_sub = rospy.Subscriber('/traffic_signal', TrafficSignal, self.traf_callback)
        rospy.Rate(10)
        print('map initiated')

    def callback(self, msg):
        self.position_z = msg.pose.position.z*1000
        self.position_x = msg.pose.position.x*1000
        self.position_yaw = 0
        self.Receivedata=1

    def traf_callback(self, msg):
        self.green_NS = msg.green_NS
        self.green_EW = msg.green_EW


    def generate_map(self, enter=0, exit=0):
        if (exit == 0):
            self.loop = True
        else:
            self.loop = False

        self.lane_width = 450
        
        self.pt = {}
        self.pt['a'] = (320, -1955)
        self.pt['b'] = (275, 200)
        self.pt['c'] = (275, 620)
        self.pt['d'] = (300, 2590)
        self.pt['e'] = (-2020, -1955)
        self.pt['f'] = (-2025, 270)
        self.pt['g'] = (-2030, 635)
        self.pt['h'] = (-2035, 2563)
        self.pt['i'] = (-2400, -1917)
        self.pt['j'] = (-2520, 276.1)
        self.pt['k'] = (-2505, 596)
        self.pt['l'] = (-2379, 2531)
        self.pt['m'] = (-4704, -1900)
        self.pt['n'] = (-4690, 135)
        self.pt['o'] = (-4662, 525)
        self.pt['p'] = (-4630, 2550)


        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.path = {}
        self.path['A'] = self.generate_line(self.pt['a'], self.pt['d'])
        self.path['B'] = self.generate_line(self.pt['e'], self.pt['h'])
        self.path['C'] = self.generate_line(self.pt['i'], self.pt['l'])
        self.path['D'] = self.generate_line(self.pt['m'], self.pt['p'])
        self.path['E'] = self.generate_line(self.pt['m'], self.pt['a'])
        self.path['F'] = self.generate_line(self.pt['n'], self.pt['b'])
        self.path['G'] = self.generate_line(self.pt['o'], self.pt['c'])
        self.path['H'] = self.generate_line(self.pt['p'], self.pt['d'])



        #data points that characterize each circle for the corners - center x, center y, radius. Each variable is a tuple (A, B, C)
        self.circle = {}
        self.circle['a'] = (self.pt['a'][0]-self.lane_width, self.pt['a'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['b'] = (self.pt['b'][0]-self.lane_width, self.pt['b'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['c'] = (self.pt['c'][0]-self.lane_width, self.pt['c'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['d'] = (self.pt['d'][0]-self.lane_width, self.pt['d'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['e'] = (self.pt['e'][0]+self.lane_width, self.pt['e'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['f'] = (self.pt['f'][0]-self.lane_width, self.pt['f'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['g'] = (self.pt['g'][0]+self.lane_width, self.pt['g'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['h'] = (self.pt['h'][0]-self.lane_width, self.pt['h'][1]+self.lane_width, self.lane_width/1.5)

        self.circle['i'] = (self.pt['i'][0]-self.lane_width, self.pt['i'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['j'] = (self.pt['j'][0]-self.lane_width, self.pt['j'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['k'] = (self.pt['k'][0]-self.lane_width, self.pt['k'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['l'] = (self.pt['l'][0]-self.lane_width, self.pt['l'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['m'] = (self.pt['m'][0]+self.lane_width, self.pt['m'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['n'] = (self.pt['n'][0]-self.lane_width, self.pt['n'][1]-self.lane_width, self.lane_width/1.5)
        self.circle['o'] = (self.pt['o'][0]+self.lane_width, self.pt['o'][1]+self.lane_width, self.lane_width/1.5)
        self.circle['p'] = (self.pt['p'][0]-self.lane_width, self.pt['p'][1]+self.lane_width, self.lane_width/1.5)


        #the ranges near each corner that activates the circle path for the limo to follow
        self.act_range = {}
        self.act_range['a'] = (self.lane_width*1.3, self.lane_width/1.3)   # entering x
        self.act_range['b'] = (self.lane_width/1.3, self.lane_width*1.3)   # entering z
        self.act_range['c'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['d'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['e'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['f'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['g'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['h'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['i'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['j'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['k'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['l'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['m'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['n'] = (self.lane_width*1.3, self.lane_width/1.3)
        self.act_range['o'] = (self.lane_width/1.3, self.lane_width*1.3)
        self.act_range['p'] = (self.lane_width*1.3, self.lane_width/1.3)


        #values of each line, each element is a tuple (kp, ki, kd)
        self.path_PID = {}
        self.path_PID['A'] = (0.0006, 0.007, 0.001)
        self.path_PID['B'] = (-0.0006, -0.007, -0.001)
        self.path_PID['C'] = (0.0006, 0.007, 0.001)
        self.path_PID['D'] = (-0.0006, -0.007, -0.001)
        self.path_PID['E'] = (0.0006, 0.007, 0.001)
        self.path_PID['F'] = (-0.0006, -0.007, -0.001)
        self.path_PID['G'] = (0.0006, 0.007, 0.001)
        self.path_PID['H'] = (-0.0006, -0.007, -0.001)

        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.circle_PID = {}
        self.circle_PID['a'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['b'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['c'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['d'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['e'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['f'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['g'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['h'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['i'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['j'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['k'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['l'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['m'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['n'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['o'] = (-0.45, -0.00045, -0.035)
        self.circle_PID['p'] = (-0.45, -0.00045, -0.035)
        



        if enter == 'o' and exit == 0 : #if the limo runs along the main path
            pts   = 'ocdheabnmilp'
            paths = 'GAHBEAFDECHD'
        else:
            pts = ''
            path = ''

        pts = list(pts)
        paths = list(paths)

        self.turning_pts = []
        self.lines = []
        self.ranges = []
        self.circles = []
        self.PIDs = []
        self.curve_PIDs = []

        for p in pts:
            self.turning_pts.append(self.pt[p])
            self.ranges.append(self.act_range[p])
            self.circles.append(self.circle[p])
            self.curve_PIDs.append(self.circle_PID[p])
        for p in paths:
            self.lines.append(self.path[p])
            self.PIDs.append(self.path_PID[p])



    #helper functions for generate_map()
    def generate_line(self, pt_1, pt_2):
        A = -(pt_2[1] - pt_1[1])
        B = -(pt_1[0] - pt_2[0])
        C = -(pt_1[1] * (pt_2[0] - pt_1[0]) - (pt_2[1] - pt_1[1]) * pt_1[0])
        return A, B, C

    def calc_distance(self, pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance

    def calc_dist_array(self, points):
        dist = []
        for i in range(len(points)-1):
            dist.append(self.calc_distance(points[i], points[i+1]))
        return dist

    #helper function for generate_map()
    def control(self,e,v_ref, eprev_lateral,eint_lateral,dt):
        if (eprev_lateral*e<=0):
            eint_lateral = 0
        kp = self.kp
        ki = self.ki
        kd = self.kd

        [ref_steer,u_k ,u_i ,u_d, eprev_lateral, eint_lateral] = self.PIDController(e, eprev_lateral, eint_lateral, dt, kp, ki, kd)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(ref_steer)
        return eprev_lateral,eint_lateral,drive_msg

    def PIDController(self, e, prev_e, prev_int, delta_t, Kp, Ki, Kd): #add theta_ref as input
        if e <= 1 and e>=-1:
            e_int = 0
        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,0.3),-0.3)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # PID controller for omega
        u_k = Kp*e
        u_i = Ki*e_int
        u_d = Kd*e_der
        u = Kp*e + Ki*e_int + Kd*e_der

        return u, u_k, u_i, u_d, e, e_int
    def steeringAngleToSteeringCommand(self,refAngle):
        x = refAngle
        y = 0.7*x
        return y
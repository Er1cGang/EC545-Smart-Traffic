#!/usr/bin/env python3
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive

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
        self.traf_sub = rospy.Subscriber('/traffic_signal', PoseStamped, self.traf_callback)
        rospy.Rate(10)
        print('map initiated')

    def callback(self, msg):
        self.position_z = msg.pose.position.z*1000
        self.position_x = msg.pose.position.x*1000
        self.position_yaw = 0
        self.Receivedata=1

    def traf_callback(self, msg):
        self.green_NS = msg.green_NS
        self.green_EW = msg.green_ES


    def generate_map(self, enter=0, exit=0):
        if (exit == 0):
            self.loop = True
        else:
            self.loop = False

        self.lane_width = 450
        
        self.pt_a = (320, -1955)
        self.pt_b = (275, 200)
        self.pt_c = (275, 580)
        self.pt_d = (300, 2590)
        self.pt_e = (-2020, -1955)
        self.pt_f = (-2025, 270)
        self.pt_g = (-2030, 635)
        self.pt_h = (-2035, 2563)
        self.pt_i = (-2400, -1917)
        self.pt_j = (-2520, 276.1)
        self.pt_k = (-2505, 596)
        self.pt_l = (-2379, 2531)
        self.pt_m = (-4704, -2000)
        self.pt_n = (-4690, 135)
        self.pt_o = (-4662, 525)
        self.pt_p = (-4630, 2550)


        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.path_A = self.generate_line(self.pt_a, self.pt_d)
        self.path_B = self.generate_line(self.pt_e, self.pt_h)
        self.path_C = self.generate_line(self.pt_i, self.pt_l)
        self.path_D = self.generate_line(self.pt_m, self.pt_p)
        self.path_E = self.generate_line(self.pt_m, self.pt_a)
        self.path_F = self.generate_line(self.pt_n, self.pt_b)
        self.path_G = self.generate_line(self.pt_o, self.pt_p)
        self.path_H = self.generate_line(self.pt_p, self.pt_d)



        #data points that characterize each circle for the corners - center x, center y, radius. Each variable is a tuple (A, B, C)
        self.circle_a = (self.pt_a[0]-self.lane_width, self.pt_a[0]+self.lane_width, self.lane_width/2)
        self.circle_b = (self.pt_b[0]-self.lane_width, self.pt_b[0]-self.lane_width, self.lane_width/2)
        self.circle_c = (self.pt_c[0]-self.lane_width, self.pt_c[0]+self.lane_width, self.lane_width/2)
        self.circle_d = (self.pt_d[0]-self.lane_width, self.pt_d[0]-self.lane_width, self.lane_width/2)
        self.circle_e = (self.pt_e[0]+self.lane_width, self.pt_e[0]+self.lane_width, self.lane_width/2)
        self.circle_f = (self.pt_f[0]-self.lane_width, self.pt_f[0]-self.lane_width, self.lane_width/2)
        self.circle_g = (self.pt_g[0]+self.lane_width, self.pt_g[0]+self.lane_width, self.lane_width/2)
        self.circle_h = (self.pt_h[0]-self.lane_width, self.pt_h[0]+self.lane_width, self.lane_width/2)

        self.circle_i = (self.pt_i[0]-self.lane_width, self.pt_i[0]+self.lane_width, self.lane_width/2)
        self.circle_j = (self.pt_j[0]-self.lane_width, self.pt_j[0]-self.lane_width, self.lane_width/2)
        self.circle_k = (self.pt_k[0]-self.lane_width, self.pt_k[0]+self.lane_width, self.lane_width/2)
        self.circle_l = (self.pt_l[0]-self.lane_width, self.pt_l[0]-self.lane_width, self.lane_width/2)
        self.circle_m = (self.pt_m[0]+self.lane_width, self.pt_m[0]+self.lane_width, self.lane_width/2)
        self.circle_n = (self.pt_n[0]-self.lane_width, self.pt_n[0]-self.lane_width, self.lane_width/2)
        self.circle_o = (self.pt_o[0]+self.lane_width, self.pt_o[0]+self.lane_width, self.lane_width/2)
        self.circle_p = (self.pt_p[0]-self.lane_width, self.pt_p[0]+self.lane_width, self.lane_width/2)


        #the ranges near each corner that activates the circle path for the limo to follow
        

        self.act_range_a = (self.lane_width*1.3, self.lane_width/1.5)   # entering x
        self.act_range_b = (self.lane_width/1.5, self.lane_width*1.3)   # entering z
        self.act_range_c = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_d = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_e = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_f = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_g = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_h = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_i = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_j = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_k = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_l = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_m = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_n = (self.lane_width*1.3, self.lane_width/1.5)
        self.act_range_o = (self.lane_width/1.5, self.lane_width*1.3)
        self.act_range_p = (self.lane_width*1.3, self.lane_width/1.5)


        #values of each line, each element is a tuple (kp, ki, kd)

        self.path_A_PID = (0.0005, 0.007, 0.001)
        self.path_B_PID = (-0.0005, -0.007, -0.001)
        self.path_C_PID = (0.0005, 0.007, 0.001)
        self.path_A_PID = (-0.0005, -0.007, -0.001)
        self.path_E_PID = (0.0005, 0.007, 0.001)
        self.path_F_PID = (-0.0005, -0.007, -0.001)
        self.path_G_PID = (0.0005, 0.007, 0.001)
        self.path_H_PID = (-0.0005, -0.007, -0.001)

        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.circle_a_PID = (-0.45, -0.00045, -0.035)
        self.circle_b_PID = (-0.45, -0.00045, -0.035)
        self.circle_c_PID = (-0.45, -0.00045, -0.035)
        self.circle_d_PID = (-0.45, -0.00045, -0.035)
        self.circle_e_PID = (-0.45, -0.00045, -0.035)
        self.circle_f_PID = (-0.45, -0.00045, -0.035)
        self.circle_g_PID = (-0.45, -0.00045, -0.035)
        self.circle_h_PID = (-0.45, -0.00045, -0.035)
        self.circle_i_PID = (-0.45, -0.00045, -0.035)
        self.circle_j_PID = (-0.45, -0.00045, -0.035)
        self.circle_k_PID = (-0.45, -0.00045, -0.035)
        self.circle_l_PID = (-0.45, -0.00045, -0.035)
        self.circle_m_PID = (-0.45, -0.00045, -0.035)
        self.circle_n_PID = (-0.45, -0.00045, -0.035)
        self.circle_o_PID = (-0.45, -0.00045, -0.035)
        self.circle_p_PID = (-0.45, -0.00045, -0.035)
        


        if enter == 'o' and exit == 0 : #if the limo runs along the main path
            #array to store all points at which the limo needs to turn, in order of traversal
            self.turning_pts = [self.pt_o, self.pt_c, self.pt_d, self.pt_h, self.pt_e, self.pt_a, self.pt_b, self.pt_n, self.pt_m, self.pt_i, self.pt_l, self.pt_p]
            #array to store all lines, in order of traversal
            self.lines = [self.path_G, self.path_A, self.path_H, self.path_B, self.path_E, self.path_A, self.path_F, self.path_D, self.path_E, self.path_C, self.path_H, self.path_D]
            #the activation range of the corners, in order of traversal
            self.ranges = [self.act_range_e, self.act_range_c, self.act_range_a, self.act_range_b]
            #array to store the circles for the corners, in order of traversal
            self.circles = [self.circle_e, self.circle_c, self.circle_a, self.circle_b]
            #array to store PID values of each line, in order of traversal, each element is a tuple (kp, ki, kd)
            self.PIDs = [self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
            #array to store PID values of each circle, in order of traversal, each element is a tuple (kp, ki, kd)
            self.curve_PIDs = [self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID]


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

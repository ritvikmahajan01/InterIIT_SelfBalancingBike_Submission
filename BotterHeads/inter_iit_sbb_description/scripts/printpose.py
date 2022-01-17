#!/usr/bin/env python

'''
Team BotterHeads

    Aryan Rajput
    Ritvik Mahajan
    Sameer Talwar
    Shreshth Mehrotra
'''


### Program for printing relevent parameters of concern

#Libraries

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, JointState
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf
import numpy as np
import math



### The names of variables, Subscribers and callback functions are same to those in controller.py




class sbb(): #Creating SBB class
    def __init__(self):

        rospy.init_node('printpose')

# Initialisations:


        #Frequency of operation.
        self.sample_rate = 100.0 #100 Hz

        #Array storing lidar distance values
        self.lidar_ranges = [0.0]*180 #180 sample points taken.

        #Pose from IMU
        self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.sbb_orientation_euler = [0.0, 0.0, 0.0] #roll,pitch,yaw values
        self.sbb_angular_velocity_bike=[0.0,0.0,0.0]
        
        #GPS parameters for pose estimation
        self.phi,self.lamb,self.h=0,0,0
        self.a = 6378137          
        self.e = 0.0818191908426
        


    #Distances and Angles for navigation.
        
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.isspawned = 0 #Used for calculating initial spawn location

        #Current coordinates
        self.x = 0.0 
        self.y = 0.0 

        #Final goal
        self.goal_x_final = 15.7
        self.goal_y_final = -40.7

        #Intermediate goal in the vicinity of the enclosure
        self.goal_x = self.goal_x_final - 2.5
        self.goal_y = self.goal_y_final + 0.448
        
        self.c = 0.0 #Coefficeint for controlling rear wheel velocity
        
        self.e_x = 0.0 #Error from goal location's x component
        self.e_y = 0.0 #Error from goal location's y component
        
        #PID parameters for navigation
        self.distance_error = 0.0 #P error
        self.distance_error_prev = 0.0
        self.distance_d = 0.0 #D error
        self.distance_i = 0.0 #I error
        
        self.handle_angle = 0.0 #Angle of the handle relative to the bike
        self.heading_angle_req = 0.0 #Angle with the x-axis , of the required direction (towards the goal)
        self.bike_angle = 0.0 #Bike yaw angle

        #Checking wether we've reached the (intermediate) goal 
        self.final = 0 

        
        
    #Geometric,Inertial properties of the bike,flywheel (for balancing)
        
        self.curr_angle = 0.0 #Roll angle of the bike, which needs to be driven to 0 for balancing
        self.moi_flywheel = 0.24 #flywheel MOI
        self.moi_bike = 4 #Bike MOI about the horizontal axis (Contributions of all parts of the bike accounted for)
        self.k_bike_over_I = 12 #(K/I) constant
        self.k_bike = self.k_bike_over_I*self.moi_bike #Bike torque constant K= (K/I)*I. Defined in such a way because (K/I) value can be computed from the IMU data. 
        self.w_flywheel = 0 #Flywheel angular velocity
        self.k_damp = 10 #Damping constant for torque control

        #stand parameters 
        self.stand1_pos = 0.0
        self.stand2_pos = 0.0
        self.stand1_w = 0.0
        self.stand2_w = 0.0

        #Subscribers
        rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
        rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/stand1/joint_state', JointState, self.stand1_callback)
        rospy.Subscriber('/stand2/joint_state', JointState, self.stand2_callback)
        rospy.Subscriber('/handle/joint_state', JointState, self.handle_callback)
        rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.lidar_callback)

    def imu_callback(self, msg):
        self.sbb_orientation_quaternion[0] = msg.orientation.x
        self.sbb_orientation_quaternion[1] = msg.orientation.y
        self.sbb_orientation_quaternion[2] = msg.orientation.z
        self.sbb_orientation_quaternion[3] = msg.orientation.w

        #conversion of orientation quarternion to euler angles
        (self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
        self.curr_angle = self.sbb_orientation_euler[1] #Bike roll angle (Balancing)
        self.bike_angle = self.sbb_orientation_euler[2] #Bike yaw angle (Navigation)
        self.sbb_angular_velocity_bike[0] = msg.angular_velocity.x #bike angular velocity (Balancing)

    def gps_callback(self, msg):
        self.phi= msg.latitude
        self.lamb= msg.longitude
        self.h=msg.altitude
        self.phi = np.deg2rad(self.phi)
        self.lamb = np.deg2rad(self.lamb)
        
        N = self.a / np.sqrt(1 - ((self.e**2) * (np.sin(self.phi)**2)))
        
        if self.isspawned == 0: #calculating initial pose

            #Converting GPS parameters to x,y values 
            self.spawn_x = (N * (1 - self.e ** 2) + self.h) * np.sin(self.phi)-1100248.62908+0.1
            self.spawn_y = -1*((N + self.h) * np.cos(self.phi) * np.sin(self.lamb)-1090835.97224+0.3)
            self.c = math.sqrt((self.goal_x - self.spawn_x)*(self.goal_x - self.spawn_x) + (self.goal_y - self.spawn_y)*(self.goal_y - self.spawn_y))
            
            self.isspawned = 1 #Initial pose calculated.

        #Converting GPS parameters to x,y values 
        self.x = (N * (1 - self.e ** 2) + self.h) * np.sin(self.phi)-1100248.62908+0.1 
        self.y = -1*((N + self.h) * np.cos(self.phi) * np.sin(self.lamb)-1090835.97224+0.3) 
    
    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges #LiDAR values

    #Stand roll angles
    def stand1_callback(self, msg):
        self.stand1_pos = msg.position[0]
    def stand2_callback(self, msg):
        self.stand2_pos = msg.position[0]

    def handle_callback(self, msg):
        self.handle_angle = msg.position[0] #Handle angle wrt bike axis



# Function to print various parameters of concern
    def displayparams(self):
        print("Current location = ("+str(self.x)+", "+str(self.y)+")")
        print("Intermediate Goal Location = ("+str(self.goal_x)+", "+str(self.goal_y)+")")
        print("Final Goal location = ("+str(self.goal_x_final)+", "+str(self.goal_y_final)+")")
        self.e_x = self.goal_x_final - self.x
        self.e_y = self.goal_y_final - self.y
        self.distance_error = math.sqrt((self.e_x*self.e_x)+(self.e_y*self.e_y))
        print("Distance from final goal = "+str(self.distance_error))
 

if __name__ == '__main__':

    sbb = sbb() #Creating SBB object
    r = rospy.Rate(sbb.sample_rate) #100Hz frequency
    while not rospy.is_shutdown():

        try:
            sbb.displayparams()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

#!/usr/bin/env python

'''
Team BotterHeads

    Aryan Rajput
    Ritvik Mahajan
    Sameer Talwar
    Shreshth Mehrotra
'''




### Program for balancing and path planning


# Libraries

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, JointState
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf
import numpy as np
import math



class sbb():  # Self Balancing Bike
    def __init__(self):

        rospy.init_node('controller')

#Initialisations:
        

        #Frequency of operation.
        self.sample_rate = 100.0 #100 Hz


    #Sensor parameters

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
        self.e_printstance_error = 0.0 #P error
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
        
        self.data_cmd = Float32()
        self.data_cmd.data = 0.0
        

    #Publishers
        self.data_flywheel = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
        self.data_rearwheel = rospy.Publisher('/drive_wheel/command', Float32, queue_size=1)
        self.data_frontwheel = rospy.Publisher('/handle/command', Float32, queue_size=1)
        self.data_stand1 = rospy.Publisher('/stand1/command', Float32, queue_size=1)
        self.data_stand2 = rospy.Publisher('/stand2/command', Float32, queue_size=1)
        
    #Subscribers
        rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
        rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/stand1/joint_state', JointState, self.stand1_callback)
        rospy.Subscriber('/stand2/joint_state', JointState, self.stand2_callback)
        rospy.Subscriber('/handle/joint_state', JointState, self.handle_callback)
        rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.lidar_callback)


#Callbacks

  
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


#Balancing
        
    def balancing(self):
        
        dt = 1.0/self.sample_rate #time interval
        w_bike=self.sbb_angular_velocity_bike[0] #Angular velocity of the bike about the x axis
        
        if abs(self.curr_angle) > 0.001: #Roll angle treated as the "error" term for control

             #PD control for torque
             #Damping constant selected for critical damping of the bike
             #vertical position achieved without oscillating
            torque  = -(self.k_bike*self.curr_angle + self.k_damp*w_bike) 

            #Incrementing flywheel angular velocity and publishing it
            
            alpha = torque/self.moi_flywheel
            self.w_flywheel += alpha*dt
            
            self.data_cmd.data = self.w_flywheel 
            self.data_flywheel.publish(self.data_cmd.data)

#Path planning

    def path_planning(self):
        if self.isspawned == 1: #initial pose calculated
            
            if self.final == 0: #Still away from (intermediate) goal

        ###The heading angle will be chosen based on LiDAR values: That direction will be chosen along which there are no obstacles nearby AND which is closest to goal

                #error in x, y
                self.e_x = self.goal_x - self.x
                self.e_y = self.goal_y - self.y

                #angle with the x axis of the vector along which the bike should move
                self.heading_angle_req = math.atan2(self.e_y,self.e_x)
                
                #I D terms for control, using distance as the error term
                self.distance_error = math.sqrt((self.e_x*self.e_x)+(self.e_y*self.e_y))
                self.distance_d = (self.distance_error-self.distance_error_prev)*self.sample_rate
                self.distance_i +=self.distance_error/self.sample_rate

                #The entire span of the LiDAR has been divided into "slots", and the corresponding heading angle is calculated based on heading_angle_req, and bike angle
                #example, if the heading_angle_req and bike_angle are approximately same, then this means the bike is aligned towards the goal and thus should continue moving straight
                #this corresponds to slot 90, which is the middle most, i.e go straight (keep heading in the current direction)
                heading_angle_slot = int(90 + ((self.heading_angle_req - self.bike_angle)*180/math.pi))

                #Checking if the region (10 degrees, 5 towards right(CW) and 5 towards left(CCW)) around the required heading angle is free of any obstacles
                #The LiDAR values correspond to the distance of an obstacle in a certain direction (There are total 180 sample points, with the 1st point corresponding to -90 degrees(CW) wrt bike axis and the 180th point corresponding to +90 degrees(CCw) wrt bike axis). A direction is considered to be clear if the obstacle distance > 18 or obstacle distance > Goal distance
                j = 0 #Temporary variable
                for i in range(heading_angle_slot-4,heading_angle_slot+5):
                    if self.lidar_ranges[i] > min(18.0, self.distance_error):
                        j +=1
                if j == 9:
                    self.heading_angle_req = math.atan2(self.e_y,self.e_x)
                    print("HQ: Path is clear. Proceed to target location.")
                else:
                    j = 0
                    right = 0 #Nearest sector to the right(CW) of direction of goal which is free
                    left = 0 #Nearest sector to the left(CCW) of direction of goal which is free
                    
                    #Calculating the left sector
                    for i in range(heading_angle_slot+1, 179):
                        if self.lidar_ranges[i] > min(18.0, self.distance_error):
                            j+=1
                        else:
                            j = 0
                        if j == 9:
                            left = i
                            break

                    #Calculating the right sector
                    j = 0
                    for i in range(heading_angle_slot, -1, -1):
                        if self.lidar_ranges[i] > min(18.0, self.distance_error):
                            j+=1
                        else:
                            j = 0
                        if j == 9:
                            right = i
                            break
                    
                    #If no sector is free in a certain direction
                    if right == 0:
                        right = 1000
                    if left == 0:
                        left = 1000

                    #Determining which sector is the closest to goal direction
                    if abs(heading_angle_slot - right) > abs(heading_angle_slot - left):
                        self.heading_angle_req = self.bike_angle + (left - 85)*math.pi/180.0
                        print("HQ: Hostiles detected on closest approach to target. Proceed to target from left")

                    else:
                        self.heading_angle_req = self.bike_angle +(right - 95)*math.pi/180.0
                        print("HQ: Hostiles detected on closest approach to target. Proceed to target from right")
                print("Heading angle required = "+str(self.heading_angle_req)+", Current Heading angle = "+str(self.bike_angle))


                if self.distance_error > 0.1: #We are still not near the goal 
                    self.data_rearwheel.publish(((5*self.distance_error)+0.5*(self.distance_d)+0.1*(self.distance_i))/self.c) #PID to control the rearwheel velocity.
                    
                    if math.fabs(self.heading_angle_req - self.bike_angle) > 0.01: #|angle| relative to bike axis by which the bike needs turn, acting as the error term
                        self.data_frontwheel.publish(-0.1*(self.heading_angle_req-self.bike_angle)) #P control to move the handle so to enable the bike to turn
                        
                    else: #body of the bike is aligned properly
                        if self.handle_angle != 0: #when the body is aligned it may be the case that the handle is rotated slightly
                            if abs(self.handle_angle) < 0.01: #handle also aligned
                                self.data_frontwheel.publish(0.0)
                            else: #handle slightly rotated, turn based on its position wrt to the bike's body ( CW or ACW)
                                self.data_frontwheel.publish(-1*(self.handle_angle)/abs(self.handle_angle)) #-ve when self.handle_angle_angle is +ve and vice versa.


                    #Lifting the stands up while navigating
                    if self.stand1_pos > -2.0:
                        self.stand1_w = -1.0
                    else:
                        self.stand1_w = 0.0
                    if self.stand2_pos < 2.0:
                        self.stand2_w = 1.0
                    else:
                        self.stand2_w = 0.0
                    self.data_stand1.publish(self.stand1_w)
                    self.data_stand2.publish(self.stand2_w)

                    
                else: #We are near the goal
                    if self.handle_angle != 0:
                            if abs(self.handle_angle) < 0.01: #handle aligned
                                self.data_frontwheel.publish(0.0)
                            else: #handle needs to be turned
                                self.data_frontwheel.publish(-1*(self.handle_angle)/abs(self.handle_angle))
                                
                    #Publishing 0 velocity to rearwheel and handle. (stopping phase)
                    self.data_rearwheel.publish(0.0)
                    self.data_frontwheel.publish(0.0)

                    #Bringing the stands down
                    if self.stand2_pos > -0.18:
                        self.stand2_w = -1
                        self.data_stand2.publish(self.stand2_w)
                    else:
                        self.data_stand2.publish(0.0)
                    if self.stand1_pos < 0.18:
                        self.stand1_w = 1
                        self.data_stand1.publish(self.stand1_w)
                    else:
                        self.data_stand1.publish(0.0)

                    #Variable accounting for the fact that we are in the vicinity of the goal.
                    self.final = 1


                #prev error for the next iteration = current error, for implementing the D term in PID control.    
                self.distance_error_prev = self.distance_error

            if self.final == 1: #near the goal
                
                #navigating to the center of the enclosure
                #earlier "goal" was a point in the vicinity of the enclosure, we will now navigate to the center(i.e inside the enclosure).
                #This is the "final goal"

                self.e_x = self.goal_x_final - self.x
                self.e_y = self.goal_y_final - self.y
                self.heading_angle_req = math.atan2(self.e_y,self.e_x)
                self.distance_error = math.sqrt((self.e_x*self.e_x)+(self.e_y*self.e_y))

                #Keeping the stands down
                if self.stand2_pos > -0.18:
                    self.stand2_w = -1
                    self.data_stand2.publish(self.stand2_w)
                else:
                    self.data_stand2.publish(0.0)
                if self.stand1_pos < 0.18:
                    self.stand1_w = 1
                    self.data_stand1.publish(self.stand1_w)
                else:
                    self.data_stand1.publish(0.0)


                if self.distance_error > 0.5: #Not quite at the final goal, keep navigating
                    
                    print("Zephyr 1 to HQ: Got visual on target. We're in the endgame now")
                    print("Heading angle required = "+str(self.heading_angle_req)+", Current Heading angle = "+str(self.bike_angle))
                    
                    self.data_rearwheel.publish(0.8*self.distance_error) # P control
                    
                    #Aligning of the bike and turning the handle
                    if math.fabs(self.heading_angle_req - self.bike_angle) > 0.01:
                        self.data_frontwheel.publish(-0.05*(self.heading_angle_req-self.bike_angle))
                    else:
                        if self.handle_angle != 0:
                            if abs(self.handle_angle) < 0.01:
                                self.data_frontwheel.publish(0.0)
                            else:
                                self.data_frontwheel.publish(-1*(self.handle_angle)/abs(self.handle_angle))

                                
                else: #Reached the final goal

                    #Aligning of the bike and turning the handle
                    if self.handle_angle != 0:
                            if abs(self.handle_angle) < 0.01:
                                self.data_frontwheel.publish(0.0)
                            else:
                                self.data_frontwheel.publish(-1*(self.handle_angle)/abs(self.handle_angle))
                    #Parking phase. Stop the rearwheel and handle.
                    self.data_rearwheel.publish(0.0)
                    self.data_frontwheel.publish(0.0)

                    #Once the stands are down, start slowing down the flywheel
                    if self.stand1_pos > 0.17 and self.stand2_pos < -0.17:
                        if abs(self.w_flywheel) > 2:
                            self.w_flywheel += -0.5*(self.w_flywheel/abs(self.w_flywheel))
                        else:
                            self.w_flywheel = 0.0 #stop the flywheel.
                        self.data_flywheel.publish(self.w_flywheel)

                        
                    print("Zypher 1 to HQ: Target acquired")
                    print("HQ: Hail Hydra!")



if __name__ == '__main__':

    sbb = sbb() #Creating SBB object
    r = rospy.Rate(sbb.sample_rate) #100Hz frequency
    while not rospy.is_shutdown():

        try:
            sbb.balancing() #Running the balancing algorithm
            sbb.path_planning() #Running the path planning algorithm
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

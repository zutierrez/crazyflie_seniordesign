#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import taskGenerator as taskg
import allocTask as taskk

import mav_msgs
import tf
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Point
from mav_msgs.msg import DroneState
from tf.transformations import quaternion_from_euler

class Crazyflie():
    def __init__(self):
        rospy.init_node("crazyflie_go")
        rospy.loginfo("Press Ctrl + C to terminate")

	taskg.init_task_generator()

	self.start_drone_info()

	self.start_assignment()


        # drone publisher
        drone_msg = DroneState()
        self.drone_pub = rospy.Publisher("crazyflie2/drone_state", DroneState, queue_size=10)
        self.rate = rospy.Rate(10) 

        # drone 1 publisher
        #drone_msg = DroneState()
        #self.drone_pub = rospy.Publisher("crazyflie2_1/drone_state", DroneState, queue_size=10)

        # drone 2 publisher
        #drone_msg = DroneState()
        #self.drone_pub = rospy.Publisher("crazyflie2_2/drone_state", DroneState, queue_size=10) 

        # drone 3 publisher
        #drone_msg = DroneState()
        #self.drone_pub = rospy.Publisher("crazyflie2_3/drone_state", DroneState, queue_size=10)
       
        # drone 4 publisher
        #drone_msg = DroneState()
        #self.drone_pub = rospy.Publisher("crazyflie2_4/drone_state", DroneState, queue_size=10) 
        
        # drone 5 publisher
        #drone_msg = DroneState()
        #self.drone_pub = rospy.Publisher("crazyflie2_5/drone_state", DroneState, queue_size=10)
        

        # subscribe to odometry
        self.pose = Point()
        self.logging_counter = 0 # logging for observing trajectory 
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("crazyflie2/odometry", Odometry, self.odom_callback)

        try:
            self.run([self.midpoint([0,0,1],[2,0,1]),np.array([2,0,1])])
	    
	    #setup multithread running for each drone here
	
	    #task assignment and execution for one drone:
	    #drone_one_go()
	    #taskUpdateFunct()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')       


    def taskUpdateFunct(self):
	#input: none
	#output: none	

	#function generates and assigns new task to drone

	while(1):
	    for x in range(0,4):
        	if(taskg.tasksAssigned[x].name == 'not assigned'):
		    taskg.tasksAssigned[x] = taskg.new_task_generator()
		    taskk.task_alloc(taskk.mrta_rank(taskg.tasksAssigned[x]),taskg.tasksAssigned[x])


    def drone_one_go(self):
	#input: none
	#output: none	

	#sends drone 1 to its next location whenever called, and updates information after reaching location

	waypoint = [midpoint(crazyflieInfo[0].postion, crazyflieInfo[0].taskAssigned.postion), np.array(crazyflieInfo[0].taskAssigned.postion)]
	run(waypoint)
	
	#need to wait until drone reaches location
	crazyflieInfo[0].updatePostion()

	#wait for drone to finish task (duration of task)

	#unassigns the task from the tasksAssigned array and from the drone
	for x in range(0,4):
	    if(tasksAssigned[x].name == crazyflieInfo[0].taskAssigned.name):
		tasksAssigned[x] = task()
		crazyflieInfo[0].taskComplete()


    def midpoint(self,currentLocation, finalLocation):
	#input: currentLocation - the current location of the drone
	#	finalLocation - the location for where the drone is being sent towards
	#output: none	

	#function generates the midpoint between current and final location

	return np.abs(np.array([(currentLocation[0]-finalLocation[0])/2, (currentLocation[1]-finalLocation[1])/2, 1]))
	    
	    
    def run(self, waypoints): 
        #waypoints = [[1, 0, 1], [2, 0, 1]] 
        for i in range(0,len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def start_drone_info(self):
	#input: none
	#output: none	

	#function assigns drones in crazyflieInfo with names

	taskk.crazyflieInfo[0].assignName('drone 1')
	taskk.crazyflieInfo[1].assignName('drone 2')
	taskk.crazyflieInfo[2].assignName('drone 3')
	taskk.crazyflieInfo[3].assignName('drone 4')
	taskk.crazyflieInfo[4].assignName('drone 5')


    def start_assignment(self):
	#input: none
	#output: none	

	#assigns the tasks to drones for first five tasks at initialization

	for x in range(0,4):
	    taskk.task_alloc(taskk.mrta_rank(self,taskg.tasksAssigned[x]), taskg.tasksAssigned[x])


    def move_to_point(self, current_waypoint, next_waypoint): 
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x trajectory conditions ------------------------
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y trajectory conditions -------------------------
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z trajectory conditions --------------------------
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0
        zv_end = 0
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw trajectory conditions ------------------------
        yawp_start = 0
        yawv_start = 0
        yawp_end = 0
        yawv_end = 0
        yawa_start = 0
        yawa_end = 0
        yawj_start = 0
        yawj_end = 0

        #compute parameters: 19 parameters, must be published 
        ax = self.polynomial_time_scaling_7th_order(xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
        ay = self.polynomial_time_scaling_7th_order(yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
        az = self.polynomial_time_scaling_7th_order(zp_start, zp_end, zv_start, zv_end, za_start, za_end, zj_start, zj_end, T)
        ayaw = self.polynomial_time_scaling_7th_order(yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)

        #publish parameters:
        t = 1/10
        for i in range(T*10):
            #x = 1.0*ax[3] + 1.0*ax[2]*(i*t) + 1.0*ax[1]*((i*t)**2) + 1.0*ax[0]*((i*t)**3)
            #y = 1.0*ax[3] + 1.0*ay[2]*(i*t) + 1.0*ay[1]*((i*t)**2) + 1.0*ay[0]*((i*t)**3)
            #z = 1.0*ax[3] + 1.0*az[2]*(i*t) + 1.0*az[1]*((i*t)**2) + 1.0*az[0]*((i*t)**3)
            #yaw = 1.0*ayaw[3] + 1.0*ayaw[2]*(i*t) + 1.0*ayaw[1]*((i*t)**2) + 1.0*ayaw[0]*((i*t)**3)

            #dx = 1.0*ax[2] + 2.0*ax[1]*(i*t) + 3.0*ax[0]*((i*t)**2)
            #dy = 1.0*ay[2] + 2.0*ay[1]*(i*t) + 3.0*ay[0]*((i*t)**2)
            #dz = 1.0*az[2] + 2.0*az[1]*(i*t) + 3.0*az[0]*((i*t)**2)
            #dyaw = 1.0*ayaw[2] + 2.0*ayaw[1]*(i*t) + 3.0*ayaw[0]*((i*t)**2)

            #d2x = 2.0*ax[1] + 6.0*ax[0]*(i*t)
            #d2y = 2.0*ay[1] + 6.0*ay[0]*(i*t)
            #d2z = 2.0*az[1] + 6.0*az[0]*(i*t)
            #d2yaw = 2.0*ayaw[1] + 6.0*ayaw[0]*(i*t)

            x = ax[7] + ax[6]*(i*t) + ax[5]*((i*t)**2) + ax[4]*((i*t)**3) + ax[3]*((i*t)**4) + ax[2]*((i*t)**5) + ax[1]*((i*t)**6) + ax[0]*((i*t)**7)       
            y = ay[7] + ay[6]*(i*t) + ay[5]*((i*t)**2) + ay[4]*((i*t)**3) + ay[3]*((i*t)**4) + ay[2]*((i*t)**5) + ay[1]*((i*t)**6) + ay[0]*((i*t)**7)      
            z = az[7] + az[6]*(i*t) + az[5]*((i*t)**2) + az[4]*((i*t)**3) + az[3]*((i*t)**4) + az[2]*((i*t)**5) + az[1]*((i*t)**6) + az[0]*((i*t)**7)
            yaw = ayaw[7] + ayaw[6]*(i*t) + ayaw[5]*((i*t)**2) + ayaw[4]*((i*t)**3) + ayaw[3]*((i*t)**4) + ayaw[2]*((i*t)**5) + ayaw[1]*((i*t)**6) + ayaw[0]*((i*t)**7) #position 
            
            dx = ax[6] + 2.0*ax[5]*(i*t) + 3.0*ax[4]*((i*t)**2) + 4.0*ax[3]*((i*t)**3) + 5.0*ax[2]*((i*t)**4) + 6.0*ax[1]*((i*t)**5) + 7.0*ax[0]*((i*t)**6)
            dy = ay[6] + 2.0*ay[5]*(i*t) + 3.0*ay[4]*((i*t)**2) + 4.0*ay[3]*((i*t)**3) + 5.0*ay[2]*((i*t)**4) + 6.0*ay[1]*((i*t)**5) + 7.0*ay[0]*((i*t)**6)
            dz = az[6] + 2.0*az[5]*(i*t) + 3.0*az[4]*((i*t)**2) + 4.0*az[3]*((i*t)**3) + 5.0*az[2]*((i*t)**4) + 6.0*az[1]*((i*t)**5) + 7.0*az[0]*((i*t)**6)
            dyaw = ayaw[6] + 2.0*ayaw[5]*(i*t) + 3.0*ayaw[4]*((i*t)**2) + 4.0*ayaw[3]*((i*t)**3) + 5.0*ayaw[2]*((i*t)**4) + 6.0*ayaw[1]*((i*t)**5) + 7.0*ayaw[0]*((i*t)**6) #velocity 
            
            d2x = 2.0*ax[5] + 6.0*ax[4]*(i*t) + 12.0*ax[3]*((i*t)**2) + 20.0*ax[2]*((i*t)**3) + 30.0*ax[1]*((i*t)**4) + 42.0*ax[0]*((i*t)**5) 
            d2y = 2.0*ay[5] + 6.0*ay[4]*(i*t) + 12.0*ay[3]*((i*t)**2) + 20.0*ay[2]*((i*t)**3) + 30.0*ay[1]*((i*t)**4) + 42.0*ay[0]*((i*t)**5)    
            d2z = 2.0*az[5] + 6.0*az[4]*(i*t) + 12.0*az[3]*((i*t)**2) + 20.0*az[2]*((i*t)**3) + 30.0*az[1]*((i*t)**4) + 42.0*az[0]*((i*t)**5) 
            d2yaw = 2.0*ayaw[5] + 6.0*ayaw[4]*(i*t) + 12.0*ayaw[3]*((i*t)**2) + 20.0*ayaw[2]*((i*t)**3) + 30.0*ayaw[1]*((i*t)**4) + 42.0*ayaw[0]*((i*t)**5) #acceleration
            

            roll = atan2(y,z) #atan2(y, sqrt( (x*x) + (z*z)))
            pitch = atan2(x,z) #atan2(x, sqrt((y*y) + (z*z)))
            q = quaternion_from_euler(roll, pitch, yaw)


            print(' x value ', x)
            print(' y value ', y)
            print(' z value ', z)
            
            print(' dx value ', dx)
            print(' dy value ', dy)
            print(' dz value ', dz)
            
            print(' roll:', roll)
            print(' pitch: ', pitch)
            print(' yaw: ', yaw)
           
            # 19 commands: drone_msg --> position (x,y,z), linear_velocity (x,y,z), linear_acceleration(x,y,z)
            # orientation(x,y,z,w), angular_velocity(x,y,z), angular_acceleration(x,y,z)
            drone_msg.position.x = x
            drone_msg.position.y = y
            drone_msg.position.z = z
            drone_msg.linear_velocity.x = dx
            drone_msg.linear_velocity.y = dy
            drone_msg.linear_velocity.z = dz
            drone_msg.linear_acceleration.x = d2x
            drone_msg.linear_acceleration.y = d2y
            drone_msg.linear_acceleration.z = d2z
            drone_msg.orientation.x = q[0]
            drone_msg.orientation.y = q[1]
            drone_msg.orientation.z = q[2]
            drone_msg.orientation.w = q[3]
            drone_msg.angular_velocity.x = roll 
            drone_msg.angular_velocity.y = pitch
            drone_msg.angular_velocity.z = dyaw
            drone_msg.angular_acceleration.x = roll
            drone_msg.angular_acceleration.y = pitch
            drone_msg.angular_acceleration.z = d2yaw
            self.drone_pub.publish(drone_msg)
            self.rate.sleep()


    def polynomial_time_scaling_7th_order(self, p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end, T):
        # input: p,v,a,j: position, velocity, acceleration and jerk of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        
        #A = np.array([[0,0,0,1], [T**3, T**2, T, 1], [0,0,1,0], [3*T**2, 2*T, 1, 0]])
        
        A = np.array([[   0,  0,  0,  0,  0,  0,  0,  1], \
                      [T**7, T**6, T**5, T**4, T**3, T**2, T, 1], \
                      [   0,  0,  0,  0,  0,  0,  1,  0], \
                      [7*(T**6), 6*(T**5), 5*(T**4), 4*(T**3), 3*(T**2), 2*T, 1, 0], \
                      [  0,  0,  0,  0,  0,  2,  0,  0], \
                      [42*(T**5), 30*(T**4), 20*(T**3), 12*(T**2), 6*T, 2, 0, 0], \
                      [  0,  0,  0,  0,  6,  0,  0,  0], \
                      [210*(T**4), 120*(T**3), 60*(T**2), 24*T, 6, 0, 0, 0]])

        A_inv = np.linalg.inv(A)
        #boundaryCond = np.array([p_start, p_end, v_start, v_end])
        boundaryCond = np.array([p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end])
        coefficients = np.dot(A_inv,boundaryCond)
        
        return coefficients


    def odom_callback(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.z = msg.pose.pose.position.z

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Crazyflie()

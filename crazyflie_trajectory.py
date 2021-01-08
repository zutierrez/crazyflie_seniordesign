#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import mav_msgs
import tf
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from mav_msgs.msg import DroneState
from tf_conversions.transformations import quarternion_from_euler

class Crazyflie():
    def __init__(self):
        rospy.init_node("crazyflie_go")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.drone_pub = rospy.Publisher("crazyflie2/drone_state", DroneState, queue_size=10)
        self.rate = rospy.Rate(10) 
        # prepare joint message to publish
        drone_msg = DroneState()

        while not rospy.is_shutdown():
            drone_msg.header.stamp = rospy.Time.now()
            self.run()
            self.rate.sleep()

    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5]]
        #for i in range(len(waypoints)-1):
        self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x trajectory conditions
        xp_start = current_waypoint[0] # x initial positon
        xp_end = next_waypoint[0] # x position (next)
        xv_start = 0 # x velocity (initial)
        xv_end = 0 # x velocity (next)
        xa_start = 0 # x acceleration (initial)
        xa_end = xa_start # x acceleration (next)
        xj_start = 0 # x jerk (initial)
        xj_end = xj_start # x jerk (next)

        # y trajectory conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = 0
        yj_start = 0
        yj_end = 0  

        # yaw trajectory conditions
        yawp_start = 0
        yawv_start = 0
        yawp_end = 0
        yawv_end = 0
        yawa_start = 0
        yawa_end = 0
        yawj_start = 0
        yawj_end = 0

        #compute parameters: 19 parameters, must be published
        ax = self.polynomial_time_scaling_7th_order(self, xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
        ay = self.polynomial_time_scaling_7th_order(self, yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
        ayaw = self.polynomial_time_scaling_7th_order(self, yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)

        #publish parameters:
        t = 1/10
        for i in range(T*10): 
            x = ax[7] + ax[6]*(i*t) + ax[5]*((i*t)**2) + ax[4]*((i*t)**3) + ax[3]*((i*t)**4) + ax[2]*((i*t)**5) + ax[1]*((i*t)**6) + ax[0]*((i*t)**7)       
            y = ay[7] + ay[6]*(i*t) + ay[5]*((i*t)**2) + ay[4]*((i*t)**3) + ay[3]*((i*t)**4) + ay[2]*((i*t)**5) + ay[1]*((i*t)**6) + ay[0]*((i*t)**7)      
            yaw = ayaw[7] + ayaw[6]*(i*t) + ayaw[5]*((i*t)**2) + ayaw[4]*((i*t)**3) + ayaw[3]*((i*t)**4) + ayaw[2]*((i*t)**5) + ayaw[1]*((i*t)**6) + ayaw[0]*((i*t)**7) #position 
            dx = ax[6] + 2*ax[5]*(i*t) + 3*ax[4]*((i*t)**2) + 4*ax[3]*((i*t)**3) + 5*ax[2]*((i*t)**4) + 6*ax[1]*((i*t)**5) + 7*ax[0]*((i*t)**6)
            dy = ay[6] + 2*ay[5]*(i*t) + 3*ay[4]*((i*t)**2) + 4*ay[3]*((i*t)**3) + 5*ay[2]*((i*t)**4) + 6*ay[1]*((i*t)**5) + 7*ay[0]*((i*t)**6)
            dyaw = ayaw[6] + 2*ayaw[5]*(i*t) + 3*ayaw[4]*((i*t)**2) + 4*ayaw[3]*((i*t)**3) + 5*ayaw[2]*((i*t)**4) + 6*ayaw[1]*((i*t)**5) + 7*ayaw[0]*((i*t)**6) #velocity 
            d2x = 2*ax[5] + 6*ax[4]*(i*t) + 12*ax[3]*((i*t)**2) + 20*ax[2]*((i*t)**3) + 30*ax[1]*((i*t)**4) + 42*ax[0]*((i*t)**5)
            d2y = 2*ay[5] + 6*ay[4]*(i*t) + 12*ay[3]*((i*t)**2) + 20*ay[2]*((i*t)**3) + 30*ay[1]*((i*t)**4) + 42*ay[0]*((i*t)**5)    
            d2yaw = 2*ayaw[5] + 6*ayaw[4]*(i*t) + 12*ayaw[3]*((i*t)**2) + 20*ayaw[2]*((i*t)**3) + 30*ayaw[1]*((i*t)**4) + 42*ayaw[0]*((i*t)**5) #acceleration
            roll = 0
            pitch = 0
            q = quarternion_from_euler(roll, pitch, yaw)
            
            # 19 commands: drone_msg --> position (x,y,z), linear_velocity (x,y,z), linear_acceleration(x,y,z)
            # orientation(x,y,z,w), angular_velocity(x,y,z), angular_acceleration(x,y,z)
            drone_msg.position.x = x
            drone_msg.position.y = y
            drone_msg.position.z = 0
            drone_msg.linear_velocity.x = dx
            drone_msg.linear_velocity.y = dy
            drone_msg.linear_velocity.z = 0
            drone_msg.linear_acceleration.x = d2x
            drone_msg.linear_acceleration.y = d2y
            drone_msg.linear_acceleration.z = 0
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
        
        A = np.array([[0,0,0,0,0,0,0,1], \
                    [T**7, T**6, T**5, T**4, T**3, T**2, T, 1], \
                    [0,0,0,0,0,0,1,0], \
                    [7*T**6, 6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0] \                    
                    [0,0,0,0,0,2,0,0] \
                    [42*T**5, 30*T**4, 20*T**3, 12*T**2, 6*T, 2, 0, 0] \
                    [0,0,0,0,6,0,0,0] \
                    [210*T**4, 120*T**3, 60*T**2, 24*T, 6, 0, 0, 0]])
        A_inv = np.linalg.inv(A)
        boundaryCond = np.array([p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end])
        coefficients = np.dot(A_inv,boundaryCond)
        
        return coefficients


    def odom_callback(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()

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
        

        self.rate = rospy.Rate(10) 

        # drone 1 publisher
        drone_msg = DroneState()
        self.drone_pub1 = rospy.Publisher("crazyflie2_1/drone_state", DroneState, queue_size=10)

        # drone 2 publisher
        drone_msg = DroneState()
        self.drone_pub2 = rospy.Publisher("crazyflie2_2/drone_state", DroneState, queue_size=10) 

        # drone 3 publisher
        drone_msg = DroneState()
        self.drone_pub3 = rospy.Publisher("crazyflie2_3/drone_state", DroneState, queue_size=10)
        
        # drone 4 publisher
        drone_msg = DroneState()
        self.drone_pub4 = rospy.Publisher("crazyflie2_4/drone_state", DroneState, queue_size=10) 
            
        # drone 5 publisher
        drone_msg = DroneState()
        self.drone_pub5 = rospy.Publisher("crazyflie2_5/drone_state", DroneState, queue_size=10)
            

        # subscribe to odometry
        #self.pose = Point()
        #self.odom_sub = rospy.Subscriber("crazyflie2/odometry", Odometry, self.odom_callback)

        self.pose1 = Point() #[x,y,z]
        self.odom_sub1 = rospy.Subscriber("crazyflie2_1/odometry", Odometry, self.odom_callback1)

        self.pose2 = Point()
        self.odom_sub2 = rospy.Subscriber("crazyflie2_2/odometry", Odometry, self.odom_callback2)

        self.pose3 = Point()
        self.odom_sub3 = rospy.Subscriber("crazyflie2_3/odometry", Odometry, self.odom_callback3)

        self.pose4 = Point()
        self.odom_sub4 = rospy.Subscriber("crazyflie2_4/odometry", Odometry, self.odom_callback4)

        self.pose5 = Point()
        self.odom_sub5 = rospy.Subscriber("crazyflie2_5/odometry", Odometry, self.odom_callback5)


        print(self.pose1)
        print(self.pose2)
        print(self.pose3)
        
        taskg.init_task_generator()

        self.start_drone_info()

        self.start_assignment()

        print("start locations")
        self.print_drone_locations()


            # ------------------------- insert task allocation + ranking -------------------
            # allocTask(): determines who wins the auction 
            # assigns a task to a drone info 

            # crazyflie2_1.task = [2,1,3]
            #task1 = [[0, 0.5, 1], [0, 1, 2]]
            #task2 = [[0, 0, 0],[0, 0, 0]]
            #task3 = [[0, 0.5, 1],[0, 0.5, 1]]
            #task4 = [[1, 2, 1],[1, 2, 1]]
            #task5 = [[-0.5, -1, 1],[-1, -2, 1]]

            ## multi threading here:

            # ranking 
            # figure out who gets a task
            # whoever gets it, we call their run function
            
            
            #--------------------------------------------------------------------------------

        try:
            self.drone_one_go()
            self.drone_two_go()
            self.drone_three_go()
            self.drone_four_go()
            self.drone_five_go()

            print("end locations")
            self.print_drone_locations()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            pass

    
    def taskUpdateFunct(self):
        #input: none
        #output: none	

        #function generates and assigns new task to drone

        while(1):
            for x in range(0,5):
                if(taskg.tasksAssigned[x].name == 'not assigned'):
                    taskg.tasksAssigned[x] = taskg.new_task_generator()
                    taskk.task_alloc(taskk.mrta_rank(taskg.tasksAssigned[x]),taskg.tasksAssigned[x])


    def print_drone_locations(self):
        print(taskk.crazyflieInfo[0].position)
        print(taskk.crazyflieInfo[1].position)
        print(taskk.crazyflieInfo[2].position)
        print(taskk.crazyflieInfo[3].position)
        print(taskk.crazyflieInfo[4].position)

        print(taskk.crazyflieInfo[0].name)
        print(taskk.crazyflieInfo[0].taskAssigned.name)

        print(taskk.crazyflieInfo[1].name)
        print(taskk.crazyflieInfo[1].taskAssigned.name)

        print(taskk.crazyflieInfo[2].name)
        print(taskk.crazyflieInfo[2].taskAssigned.name)

        print(taskk.crazyflieInfo[3].name)
        print(taskk.crazyflieInfo[3].taskAssigned.name)

        print(taskk.crazyflieInfo[4].name)
        print(taskk.crazyflieInfo[4].taskAssigned.name)

        print(taskk.crazyflieInfo[0].taskAssigned.position)
        print(taskk.crazyflieInfo[1].taskAssigned.position)
        print(taskk.crazyflieInfo[2].taskAssigned.position)
        print(taskk.crazyflieInfo[3].taskAssigned.position)
        print(taskk.crazyflieInfo[4].taskAssigned.position)
        


    def drone_one_go(self):
        #input: none
        #output: none	

        #sends drone 1 to its next location whenever called, and updates information after reaching location

        waypoint = [self.midpoint(taskk.crazyflieInfo[0].position, taskk.crazyflieInfo[0].taskAssigned.position), np.array(taskk.crazyflieInfo[0].taskAssigned.position)]
        self.run1(waypoint)
        
        #need to wait until drone reaches location
        taskk.crazyflieInfo[0].updatePosition()

        #wait for drone to finish task (duration of task)

        #unassigns the task from the tasksAssigned array and from the drone
        for x in range(0,5):
            if(taskg.tasksAssigned[x].name == taskk.crazyflieInfo[0].taskAssigned.name):
                taskg.tasksAssigned[x] = taskk.task()
                taskk.crazyflieInfo[0].taskComplete()

    def drone_two_go(self):
        #input: none
        #output: none	

        #sends drone 1 to its next location whenever called, and updates information after reaching location

        waypoint = [self.midpoint(taskk.crazyflieInfo[1].position, taskk.crazyflieInfo[1].taskAssigned.position), np.array(taskk.crazyflieInfo[1].taskAssigned.position)]
        self.run2(waypoint)
        
        #need to wait until drone reaches location
        taskk.crazyflieInfo[1].updatePosition()

        #wait for drone to finish task (duration of task)

        #unassigns the task from the tasksAssigned array and from the drone
        for x in range(0,5):
            if(taskg.tasksAssigned[x].name == taskk.crazyflieInfo[1].taskAssigned.name):
                taskg.tasksAssigned[x] = taskk.task()
                taskk.crazyflieInfo[1].taskComplete()

    def drone_three_go(self):
        #input: none
        #output: none	

        #sends drone 1 to its next location whenever called, and updates information after reaching location

        waypoint = [self.midpoint(taskk.crazyflieInfo[2].position, taskk.crazyflieInfo[2].taskAssigned.position), np.array(taskk.crazyflieInfo[2].taskAssigned.position)]
        self.run3(waypoint)
        
        #need to wait until drone reaches location
        taskk.crazyflieInfo[2].updatePosition()

        #wait for drone to finish task (duration of task)

        #unassigns the task from the tasksAssigned array and from the drone
        for x in range(0,5):
            if(taskg.tasksAssigned[x].name == taskk.crazyflieInfo[2].taskAssigned.name):
                taskg.tasksAssigned[x] = taskk.task()
                taskk.crazyflieInfo[2].taskComplete()

    def drone_four_go(self):
        #input: none
        #output: none	

        #sends drone 1 to its next location whenever called, and updates information after reaching location

        waypoint = [self.midpoint(taskk.crazyflieInfo[3].position, taskk.crazyflieInfo[3].taskAssigned.position), np.array(taskk.crazyflieInfo[3].taskAssigned.position)]
        self.run4(waypoint)
        
        #need to wait until drone reaches location
        taskk.crazyflieInfo[3].updatePosition()

        #wait for drone to finish task (duration of task)

        #unassigns the task from the tasksAssigned array and from the drone
        for x in range(0,5):
            if(taskg.tasksAssigned[x].name == taskk.crazyflieInfo[3].taskAssigned.name):
                taskg.tasksAssigned[x] = taskk.task()
                taskk.crazyflieInfo[3].taskComplete()

    def drone_five_go(self):
        #input: none
        #output: none	

        #sends drone 1 to its next location whenever called, and updates information after reaching location

        waypoint = [self.midpoint(taskk.crazyflieInfo[4].position, taskk.crazyflieInfo[4].taskAssigned.position), np.array(taskk.crazyflieInfo[4].taskAssigned.position)]
        self.run5(waypoint)
        
        #need to wait until drone reaches location
        taskk.crazyflieInfo[4].updatePosition()

        #wait for drone to finish task (duration of task)

        #unassigns the task from the tasksAssigned array and from the drone
        for x in range(0,5):
            if(taskg.tasksAssigned[x].name == taskk.crazyflieInfo[4].taskAssigned.name):
                taskg.tasksAssigned[x] = taskk.task()
                taskk.crazyflieInfo[4].taskComplete()


    def midpoint(self, currentLocation, finalLocation):
        #input: currentLocation - the current location of the drone
        #	finalLocation - the location for where the drone is being sent towards
        #output: none	

        #function generates the midpoint between current and final location

        return np.abs(np.array([(currentLocation[0]-finalLocation[0])/2, (currentLocation[1]-finalLocation[1])/2, 1]))
	    

    def start_drone_info(self):
        #input: none
        #output: none	

        #function assigns drones in crazyflieInfo with names

        taskk.crazyflieInfo[0].assignName('alpha')
        pose = [self.pose1.x, self.pose1.y, self.pose1.z]
        taskk.crazyflieInfo[0].setPosition(pose)

        taskk.crazyflieInfo[1].assignName('beta')
        pose = [self.pose2.x, self.pose2.y, self.pose2.z]
        taskk.crazyflieInfo[1].setPosition(pose)

        taskk.crazyflieInfo[2].assignName('charlie')
        pose = [self.pose3.x, self.pose3.y, self.pose3.z]
        taskk.crazyflieInfo[2].setPosition(pose)

        taskk.crazyflieInfo[3].assignName('delta')
        pose = [self.pose4.x, self.pose4.y, self.pose4.z]
        taskk.crazyflieInfo[3].setPosition(pose)

        taskk.crazyflieInfo[4].assignName('echo')
        pose = [self.pose5.x, self.pose5.y, self.pose5.z]
        taskk.crazyflieInfo[4].setPosition(pose)

    def start_assignment(self):
        #input: none
        #output: none	

        #assigns the tasks to drones for first five tasks at initialization

        for x in range(0,5):
            taskk.task_alloc(taskk.mrta_rank(taskg.tasksAssigned[x]), taskg.tasksAssigned[x])





    def run1(self, waypoints): 
        print("drone 1 waypoint:")	
        print(waypoints)
        drone_msg = DroneState()
        for i in range(len(waypoints)-1):
            self.move_to_point1(waypoints[i], waypoints[i+1])

    def run2(self, waypoints): 
        drone_msg = DroneState()
        for i in range(len(waypoints)-1):
            self.move_to_point2(waypoints[i], waypoints[i+1])

    def run3(self, waypoints): 
        drone_msg = DroneState()
        for i in range(len(waypoints)-1):
            self.move_to_point3(waypoints[i], waypoints[i+1])

    def run4(self, waypoints): 
        drone_msg = DroneState()
        for i in range(len(waypoints)-1):
            self.move_to_point4(waypoints[i], waypoints[i+1])


    def run5(self, waypoints): 
        drone_msg = DroneState()
        for i in range(len(waypoints)-1):
            self.move_to_point5(waypoints[i], waypoints[i+1])    

        # pose = np.array([self.pose1.x, self.pose1.y, self.pose1.z]) # must change this later for multiple drones
        # goal = np.array(waypoints[-1])
        # print(pose)
        # print(goal)
        
        # squared_dist = np.sum((pose-waypoints[-1])**2, axis = 0)
        # dist = sqrt(squared_dist)
        # print(dist)
        # if dist <= 4:
        #     print("Publishing zeros to controller now .... ")
        #     roll = 0
        #     pitch = 0
        #     yaw = 0
        #     q = quaternion_from_euler(roll, pitch, yaw)
        #     drone_msg.position.x = 0
        #     drone_msg.position.y = 0
        #     drone_msg.position.z = 0
        #     drone_msg.linear_velocity.x = 0
        #     drone_msg.linear_velocity.y = 0
        #     drone_msg.linear_velocity.z = 0
        #     drone_msg.linear_acceleration.x = 0
        #     drone_msg.linear_acceleration.y = 0
        #     drone_msg.linear_acceleration.z = 0
        #     drone_msg.orientation.x = 0
        #     drone_msg.orientation.y = 0
        #     drone_msg.orientation.z = 0
        #     drone_msg.orientation.w = 0
        #     drone_msg.angular_velocity.x = roll 
        #     drone_msg.angular_velocity.y = pitch
        #     drone_msg.angular_velocity.z = 0
        #     drone_msg.angular_acceleration.x = roll
        #     drone_msg.angular_acceleration.y = pitch
        #     drone_msg.angular_acceleration.z = 0
        #     self.drone_pub.publish(drone_msg)
        #     self.rate.sleep()


    def move_to_point1(self, current_waypoint, next_waypoint): 
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x boundary conditions 
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0.4
        zv_end = 0.4
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
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
            
            roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
            pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
            q = quaternion_from_euler(roll, pitch, yaw)
           
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
            self.drone_pub1.publish(drone_msg)
            self.rate.sleep()

    def move_to_point2(self, current_waypoint, next_waypoint): 
        drone_msg = DroneState()
        T = 2

        # x boundary conditions 
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0.4
        zv_end = 0.4
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
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
            
            roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
            pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
            q = quaternion_from_euler(roll, pitch, yaw)
           
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
            self.drone_pub2.publish(drone_msg) # drone_pub2 publisher for crazyflie2_2
            self.rate.sleep()

    def move_to_point3(self, current_waypoint, next_waypoint): 
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x boundary conditions 
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0.4
        zv_end = 0.4
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
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
            
            roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
            pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
            q = quaternion_from_euler(roll, pitch, yaw)
           
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
            self.drone_pub3.publish(drone_msg) # drone_pub3 publisher for crazyflie2_3
            self.rate.sleep()

    def move_to_point4(self, current_waypoint, next_waypoint): 
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x boundary conditions 
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0.4
        zv_end = 0.4
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
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
            
            roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
            pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
            q = quaternion_from_euler(roll, pitch, yaw)
           
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
            self.drone_pub4.publish(drone_msg) # drone_pub4 publisher for crazyflie2_4
            self.rate.sleep()

    def move_to_point5(self, current_waypoint, next_waypoint): 
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        drone_msg = DroneState()
        T = 2

        # x boundary conditions 
        xp_start = current_waypoint[0] # positon (inital)
        xp_end = next_waypoint[0]      # position (next)
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yp_start = current_waypoint[1]
        yp_end = next_waypoint[1]
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zp_start = current_waypoint[2]
        zp_end = next_waypoint[2]
        zv_start = 0.4
        zv_end = 0.4
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
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
            
            roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
            pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
            q = quaternion_from_euler(roll, pitch, yaw)
           
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
            self.drone_pub5.publish(drone_msg) # drone_pub5 publisher for crazyflie2_5
            self.rate.sleep()


    def polynomial_time_scaling_7th_order(self, p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end, T):
        # input: p,v,a,j: position, velocity, acceleration and jerk of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        
        A = np.array([[   0,  0,  0,  0,  0,  0,  0,  1], \
                      [T**7, T**6, T**5, T**4, T**3, T**2, T, 1], \
                      [   0,  0,  0,  0,  0,  0,  1,  0], \
                      [7*(T**6), 6*(T**5), 5*(T**4), 4*(T**3), 3*(T**2), 2*T, 1, 0], \
                      [  0,  0,  0,  0,  0,  2,  0,  0], \
                      [42*(T**5), 30*(T**4), 20*(T**3), 12*(T**2), 6*T, 2, 0, 0], \
                      [  0,  0,  0,  0,  6,  0,  0,  0], \
                      [210*(T**4), 120*(T**3), 60*(T**2), 24*T, 6, 0, 0, 0]])

        A_inv = np.linalg.inv(A)
        boundaryCond = np.array([p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end])
        coefficients = np.dot(A_inv,boundaryCond)
        
        return coefficients

    def mrta_rank(self,taskToRank):
        pass
    # input: position of task class object, position of 5 crazyflies
    # output: rank [rank1, rank2, rank3, rank4, rank5]
    
    # assuming task_position is given as np.array(x,y,z)
    # assuming crazyflie_position is given as np.array(self.pose.x,self.pose.y,self.pose.z)
    
    #goes through and ranks drones based on certain characteristics of drones
    #for x in range(0,4):
        #finds the distance of all drones to the task
        #ranks distances based on the max possible 
        #dist[x]
        
       #boundaryLength = 10 #The size of the sides of the area for the tasks
        #maxDistance = sqrt(2*boundaryLength*boundaryLength) #max possible distance away a drone could be

        #dist = np.linalg.norm(taskToRank - self.pose) 
        #dist["dist{0}".format(x)] = np.linalg.norm(taskToRank.position - crazyflieInfo(x).position)
       # distRank = 1 - (dist/maxDistance)
        #rankDist["rankDist{0}".format(x)] = 1 - dist(x)/maxDistance
    
        #higher rank for task if parameters match for the task
        #if(taskToRank.robotParameter == crazyflieInfo(x).robotParameter)
            #rankParam["rankParam{0}.format(x)] = 1
        #else
            #rankParam["rankParam{0}.format(x)] = 0.5
        
        #higher rank for task if not doing anything
        #if(crazyflieInfo(x).taskAssigned == 'no task')
            #rankDoingNothing["rankDoingNothing{0}.format(x)] = 1
        #else    
            #rankDoingNothing["rankDoingNothing{0}.format(x)] = 0
            
        #numRankingFactors = 3
    
        #inverts the values so low numbers mean higher ranking
    #overallRank0 = numRankingFactors - rankDist0 + rankParam0 + rankDoingNothing0
    #overallRank1 = numRankingFactors - rankDist1 + rankParam1 + rankDoingNothing1
    #overallRank2 = numRankingFactors - rankDist2 + rankParam2 + rankDoingNothing2
    #overallRank3 = numRankingFactors - rankDist3 + rankParam3 + rankDoingNothing3
    #overallRank4 = numRankingFactors - rankDist4 + rankParam4 + rankDoingNothing4
    

    # also can consider the crazyflie's specific reward associated with the task
    # reward_vector = [reward1, .... rewardn] (0 - 1)
    #pre_rank = reward*dist #element-wise multiplication
    
    # https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.sort.html
    #dtype = [('drone','S10'), ('distance', float),('rankValue',float)]
    #drones = [('alpha', dist0, overallRank0) ('beta',dist1, overallRank1), ('charlie',dist2, overallRank2), ('delta',dist3, overallRank3), ('echo',dis4, overallRank4)]
    #pre_ranked = np.array(drones, dtype=dtype)
    
    #rank = np.sort(pre_ranked, order='rankValue')
        #rank = 0
	    

    def odom_callback1(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose1.x = msg.pose.pose.position.x
        self.pose1.y = msg.pose.pose.position.y
        self.pose1.z = msg.pose.pose.position.z


    def odom_callback2(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose2.x = msg.pose.pose.position.x
        self.pose2.y = msg.pose.pose.position.y
        self.pose2.z = msg.pose.pose.position.z


    def odom_callback3(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose3.x = msg.pose.pose.position.x
        self.pose3.y = msg.pose.pose.position.y
        self.pose3.z = msg.pose.pose.position.z


    def odom_callback4(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose4.x = msg.pose.pose.position.x
        self.pose4.y = msg.pose.pose.position.y
        self.pose4.z = msg.pose.pose.position.z


    def odom_callback5(self, msg): 
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose5.x = msg.pose.pose.position.x
        self.pose5.y = msg.pose.pose.position.y
        self.pose5.z = msg.pose.pose.position.z


if __name__ == '__main__':
    whatever = Crazyflie()

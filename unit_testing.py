#https://docs.python.org/3/library/unittest.html
#http://wiki.ros.org/unittest
#https://pythontesting.net/framework/unittest/unittest-introduction/
#http://docs.ros.org/en/diamondback/api/nav_msgs/html/msg/Odometry.html odometry node format

#!/usr/bin/env python
PKG = 'test_roslaunch'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
import rospy
from nav_msgs.msg import Odometry
import sys, unittest, time, random
import numpy as np
import allocTask
import taskGenerator

def genWaypoints(num_waypoints): #randomly generates waypoints
    random.seed(time.time())
    
    waypoints = np.empty(num_waypoints)
    
    for i in range(num_waypoints):
        x = random.randrange(1,100,1)
        y = random.randrange(1,100,1)
        z = random.randrange(1,100,1)
        
        random_xyz = np.array([[x,y,z]])
        np.append(waypoints,[random_xyz],axis=0)
        
    return waypoints
    
def checkWaypoints(next_waypoint)

    current_waypoint = self.pose #update waypoint to current pos
    
    move_to_point(self, current_waypoint, next_waypoint) #move to next waypoint
    current_waypoint = self.pose #update current waypoint to pos
    
    return current_waypoint #return current position
    
def genTasks(num_tasks):
    robotParams = [ #define robot params as defined in taskGenerator
    'robotParamOne',
    'robotParamTwo',
    'robotParamThree'
    ]

    random.seed(time.time())
    
    tasks = np.empty(num_tasks)
    
    for i in range(num_tasks):
        tasks[i] = allocTask.task('task ' i, random.randrange(0,10), random.randrange(0,10), 1, robotParams = random.sample(range(3),3), random.randrange(1,5) * 5) #randomly generate tasks
        
    return tasks

class TestTrajectory(unittest.TestCase):
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)


     def test_move_to_point(self):
        for i in range(waypoint):
         self.assertAlmostEquals(checkWaypoints(), self.pose, 0.05) #check that the waypoint is equal to current drone pos, +/- 5%
 
 if __name__ == '__main__':
     import rostest
     rostest.rosrun(PKG, 'test_move_to_point', TestTrajectory)
     
class TestTaskAlloc(unittest.TestCase):
    tasks = genTasks(20) #Generate 20 sample tasks

    for i in range(tasks): 
        print(tasks(i)) #print each task generated
        
    def test_task_alloc(self):
        for i in range(tasks):
            ranked_output = mrta_rank(tasks[i]) #begin ranking
            print(ranked_output)
            task_alloc(ranked_output,tasks[i]) #allocate tasks
            self.assertEquals(tasks[i], ranked_output) #ensure the ranked task is equivalent to its proper rank
        
 if __name__ == '__main__':
     import rostest
     rostest.rosrun(PKG, 'test_task_alloc', TestTaskAlloc)

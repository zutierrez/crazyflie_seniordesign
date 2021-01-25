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

def genWaypoints(num_waypoints):
    random.seed(time.time())
    
    waypoints = np.empty(0)
    
    for i in range(num_waypoints):
        x = random.randrange(1,100,1)
        y = random.randrange(1,100,1)
        z = random.randrange(1,100,1)
        
        random_xyz = np.array([[x,y,z]])
        np.append(waypoints,[random_xyz],axis=0)
        
    return waypoints
    
def checkWaypoints(next_waypoint)

    current_waypoint = self.pose
    
    move_to_point(self, current_waypoint, next_waypoint)
    current_waypoint = self.pose
    
    return current_waypoint

class TestTrajectory(unittest.TestCase):
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)


     def test_move_to_point(self):
        for i in range(waypoint):
         self.assertAlmostEquals(checkWaypoints(), self.pose)
 
 if __name__ == '__main__':
     import rostest
     rostest.rosrun(PKG, 'test_move_to_point', TestTrajectory)
     

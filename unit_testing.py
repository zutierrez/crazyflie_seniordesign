#!/usr/bin/env python
PKG = 'test_roslaunch'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
 
import sys, unittest,time, random
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

class TestTrajectory(unittest.TestCase):

     def test_move_to_point(self):
         self.assertEquals(move_to_point(self, current_waypoint, next_waypoint), next_waypoint)
 
 if __name__ == '__main__':
     import rostest
     rostest.rosrun(PKG, 'test_move_to_point', TestTrajectory)
     
#https://docs.python.org/3/library/unittest.html
#http://wiki.ros.org/unittest
#https://pythontesting.net/framework/unittest/unittest-introduction/
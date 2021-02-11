import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

#Topic: crazyflie2_n/Odometry
#nav_msgs/Odometry.msg

class robotInfo:
	def __init__(self, name = '', position_x = 0, position_y = 0, position_z = 1, robotParameter = 0):
		self.name = name
		self.position = np.array([position_x, position_y, position_z])
		self.taskAssigned = task()  #initializes to a blank task
        	self.taskRank = 0
		self.enabled = 1
		self.robotParameter = robotParameter
    	#def __init__(self):
        #	self.name = ''
	#	self.position = [0,0,0]
	#	self.taskAssigned.task()  #initializes to a blank task
        #	self.taskRank = 0
	#	self.enabled = 0
	#	self.robotParameter = 0
    	def assignTask(self, newTaskInfo, newTaskRank):
		#assigns a task with its ranking to the drones info
        	self.taskAssigned = newTaskInfo
        	self.taskRank = newTaskRank
		self.enabled = 1
	def assignName(self, newName):
		#call this function to change the drones name
		self.name = newName
	def updatePostion(self):
		#call this function once the drone reaches task location
		self.position = taskAssigned.position
	def taskComplete(self):
		#clears the task from the drones info
		self.taskAssinged.task()



class task:
	def __init__(self, name = 'not assigned', position_x = 0, position_y = 0, position_z = 0, robotParameter = 0, taskLength = 0):
		self.name = name
		self.position = np.array([position_x, position_y, position_z])
		self.robotParameter = robotParameter
        	self.taskLength = taskLength
   	#def _init_(self):
        #	self.name = 'not assigned'
        #	self.position = [0,0,0]
       	 #	self.robotParameter = 0
        #	self.taskLength = 0
	


crazyflieInfo = [robotInfo() for i in range(0,5)]

boundaryLength = 10 #The size of the sides of the area for the tasks
maxDistance = sqrt(2*boundaryLength*boundaryLength) #max possible distance away a drone could be

def mrta_rank(self,taskToRank):
    # input: position of task class object, position of 5 crazyflies
    # output: rank [rank1, rank2, rank3, rank4, rank5]
    
    # assuming task_position is given as np.array(x,y,z)
    # assuming crazyflie_position is given as np.array(self.pose.x,self.pose.y,self.pose.z)
    
    #goes through and ranks drones based on certain characteristics of drones

    dist = [0,0,0,0,0]
    rankDist = [0,0,0,0,0]
    rankParam = [0,0,0,0,0]
    rankDoingNothing = [0,0,0,0,0]

    for x in range (0,4):
        #finds the distance of all drones to the task
        #ranks distances based on the max possible distance
        dist[x] = np.linalg.norm(taskToRank.position - crazyflieInfo[x].position)
        rankDist[x] = 1 - dist[x]/maxDistance
    
        #higher rank for task if parameters match for the task
        if(taskToRank.robotParameter == crazyflieInfo[x].robotParameter):
            rankParam[x] = 1
        else:
            rankParam[x] = 0.5
        
        #higher rank for task if not doing anything
        if(crazyflieInfo[x].taskAssigned == 'not assigned'):
            rankDoingNothing[x] = 1
        else:
            rankDoingNothing[x] = 0
            
    numRankingFactors = 3
    
    #inverts the values so low numbers mean higher ranking
    overallRank0 = numRankingFactors - rankDist[0] + rankParam[0] + rankDoingNothing[0]
    overallRank1 = numRankingFactors - rankDist[1] + rankParam[1] + rankDoingNothing[1]
    overallRank2 = numRankingFactors - rankDist[2] + rankParam[2] + rankDoingNothing[2]
    overallRank3 = numRankingFactors - rankDist[3] + rankParam[3] + rankDoingNothing[3]
    overallRank4 = numRankingFactors - rankDist[4] + rankParam[4] + rankDoingNothing[4]
    

    # also can consider the crazyflie's specific reward associated with the task
    # reward_vector = [reward1, .... rewardn] (0 - 1)
    #pre_rank = reward*dist #element-wise multiplication
    
    # https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.sort.html
    dtype = [('drone','S10'), ('distance', float), ('rankValue',float)]
    drones = [('alpha', dist[0], overallRank0), ('beta',dist[1], overallRank1), ('charlie',dist[2], overallRank2), ('delta',dist[3], overallRank3), ('echo',dist[4], overallRank4)]
    pre_ranked = np.array(drones, dtype=dtype)
    
    rank = np.sort(pre_ranked, order='rankValue')

    return rank


def task_alloc(rank,taskToAlloc):
    #input: rank vector each crazyflie, task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task
    
    #unsure what will happen if we try to assign too many tasks at one time
    
    #see if first ranked drone is free, and assign task if so
    for x in range (0,4):
        for y in range (0,4):
            if(crazyflieInfo[y].name == rank[x][0]):
                if(crazyflieInfo[y].taskAssigned == 'not assigned'):
                    crazyflieInfo[y].assignTask(taskToAlloc, rank[x][2])
                else:   #if the drones current task has a rank value of 1 less than the new task, it switches tasks ----- switching policy here
                    if(crazyflieInfo[y].taskRank + 1 < rank[x][2]):
                        oldTaskToReassign = crazyflieInfo[y].taskAssigned
                        crazyflieInfo[y].assignTask(taskToAlloc, rank[x][2])
                        #line below calls the task allocation for the task that was just taken from the other drone
                        task_alloc(mrta_rank(oldTaskToReassign),oldTaskToReassign)
                        
                        

#to be used to compare as a more basic assignment versus the more complex version above
def domino_assign(taskToAlloc):
    #input: task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task
    
    #must make sure there are not more tasks being assigned than drones available
    
    #goes through drones in order until a drone is found to do the task
    for x in range (0,4):
        if(crazyflieInfo[x].taskAssigned == 'no Task'):
            crazyflieInfo[x].assignTask(taskToAlloc)

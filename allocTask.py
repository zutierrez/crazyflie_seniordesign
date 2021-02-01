import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

#Topic: crazyflie2_n/Odometry
#nav_msgs/Odometry.msg

crazyflieInfo = [robotInfo() for i in range(0,4)]

boundaryLength = 10 #The size of the sides of the area for the tasks
maxDistance = sqrt(2*boundaryLength*boundaryLength) #max possible distance away a drone could be

def mrta_rank(self,taskToRank):
    # input: position of task class object, position of 5 crazyflies
    # output: rank [rank1, rank2, rank3, rank4, rank5]
    
    # assuming task_position is given as np.array(x,y,z)
    # assuming crazyflie_position is given as np.array(self.pose.x,self.pose.y,self.pose.z)
    
    #goes through and ranks drones based on certain characteristics of drones
    for(x in range (0,4)):
        #finds the distance of all drones to the task
        #ranks distances based on the max possible distance
        dist["dist{0}".format(x)] = np.linalg.norm(taskToRank.position - crazyflieInfo(x).position)
        rankDist["rankDist{0}".format(x)] = 1 - dist(x)/maxDistance
    
        #higher rank for task if parameters match for the task
        if(taskToRank.robotParameter == crazyflieInfo(x).robotParameter)
            rankParam["rankParam{0}.format(x)] = 1
        else
            rankParam["rankParam{0}.format(x)] = 0.5
        
        #higher rank for task if not doing anything
        if(crazyflieInfo(x).taskAssigned == 'no task')
            rankDoingNothing["rankDoingNothing{0}.format(x)] = 1
        else    
            rankDoingNothing["rankDoingNothing{0}.format(x)] = 0
            
    numRankingFactors = 3
    
    #inverts the values so low numbers mean higher ranking
    overallRank0 = numRankingFactors - rankDist0 + rankParam0 + rankDoingNothing0
    overallRank1 = numRankingFactors - rankDist1 + rankParam1 + rankDoingNothing1
    overallRank2 = numRankingFactors - rankDist2 + rankParam2 + rankDoingNothing2
    overallRank3 = numRankingFactors - rankDist3 + rankParam3 + rankDoingNothing3
    overallRank4 = numRankingFactors - rankDist4 + rankParam4 + rankDoingNothing4
    

    # also can consider the crazyflie's specific reward associated with the task
    # reward_vector = [reward1, .... rewardn] (0 - 1)
    pre_rank = reward*dist #element-wise multiplication
    
    # https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.sort.html
    dtype = [('drone','S10'), ('distance', float),('rankValue',float)]
    drones = [('alpha', dist0, overallRank0) ('beta',dist1, overallRank1), ('charlie',dist2, overallRank2), ('delta',dist3, overallRank3), ('echo',dis4, overallRank4)]
    pre_ranked = np.array(drones, dtype=dtype)
    
    rank = np.sort(pre_ranked, order='rankValue')

	return rank

class robotInfo:
	def _init_(self, name, position_x, position_y, position_z, robotParameter)
		self.name = name
		self.position = [position_x, position_y, position_z]
		self.taskAssigned = 'no task'  #give name of tasks here when assigned
        self.taskRank = ''
		self.enabled = 1
		self.robotParameter = robotParameter
    def _init_(self)
        self.name = ''
		self.position = [0,0,0]
		self.taskAssigned = ''  #give name of tasks here when assigned
        self.taskRank = ''
		self.enabled = 0
		self.robotParameter = ''
    def assignTask(self, newTaskInfo, newTaskRank)
        self.taskAssigned = newTaskInfo
        self.taskRank = newTaskRank



class task:
	def _init_(self, name, position_x, position_y, position_z, robotParameter, taskLength)
		self.name = name
		self.position = [position_x, position_y, position_z]
		self.robotParameter = robotParameter
        self.taskLength = taskLength
    def _init_(self)
        self.name = 'not assigned'
        self.postion = [0,0,0]
        self.robotParameter = 0
        self.taskLength = 0

def task_alloc(rank,taskToAlloc):
    #input: rank vector each crazyflie, task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task
    
    #unsure what will happen if we try to assign too many tasks at one time
    
    #see if first ranked drone is free, and assign task if so
    for(x in range (0,4)):
        for(y in range (0,4))
            if(crazyflieInfo(y).name == rank(x).drone)
                if(crazyflieInfo(y).taskAssigned == 'no task')
                    crazyflieInfo(y).assignTask(taskToAlloc, rank(x).rankValue)
                else        #if the drones current task has a rank value of 1 less than the new task, it switches tasks ----- switching policy here
                    if(crazyflieInfo(y).taskRank + 1 < rank(x).rankValue)
                        oldTaskToReassign = crazyflieInfo(y).taskAssigned
                        crazyflieInfo(y).assignTask(taskToAlloc, rank(x).rankValue)
                        #line below calls the task allocation for the task that was just taken from the other drone
                        task_alloc(mrta_rank(oldTaskToReassign),oldTaskToReassign)
                        
                        

#to be used to compare as a more basic assignment versus the more complex version above
def domino_assign(taskToAlloc)
    #input: task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task
    
    #must make sure there are not more tasks being assigned than drones available
    
    #goes through drones in order until a drone is found to do the task
    for(x in range (0,4))
        if(crazyflieInfo(x).taskAssigned == 'no Task')
            crazyflieInfo(x).assignTask(taskToAlloc)

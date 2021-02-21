import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

class robotInfo:
    def __init__(self, name = '', position_x = 0, position_y = 0, position_z = 1, robotParameter = 0):
        self.name = name
        self.position = np.array([position_x, position_y, position_z])
        self.taskAssigned = task() #initializes to a blank task
        self.taskRank = 0
        self.robotParameter = robotParameter

    def assignTask(self, newTaskInfo, newTaskRank):
        #assigns a task with its ranking to the drones info
        self.taskAssigned = newTaskInfo
        self.taskRank = newTaskRank
	
    def assignName(self, newName):
        #call this function to change the drones name
        self.name = newName
	
    def updatePosition(self):
        #call this function once the drone reaches task location
        self.position = self.taskAssigned.position
	
    def taskComplete(self):
        #clears the task from the drones info
        self.taskAssigned = task()
	
    def setPosition(self, position):
        self.position = position

class task:
    def __init__(self, name = 'not assigned', position_x = 0, position_y = 0, position_z = 0, robotParameter = 0, taskLength = 0):
        self.name = name
        self.position = np.array([position_x, position_y, position_z])
        self.robotParameter = robotParameter
        self.taskLength = taskLength
	

crazyflieInfo = [robotInfo() for i in range(0,5)] #vector of CrazyflieInfo data members
boundaryLength = 10 #the size of the task area
maxDistance = sqrt(2*boundaryLength*boundaryLength) #max possible distance away a drone could be

def mrta_rank(taskToRank):
    # input: position of task class object, position of 5 crazyflies
    # output: rank vector: [rank1, rank2, rank3, rank4, rank5]

    dist = [0,0,0,0,0]
    rankDist = [0,0,0,0,0]
    rankParam = [0,0,0,0,0]
    rankDoingNothing = [0,0,0,0,0]

    for x in range (0,5): #computes the distance of all drones to tasks and ranks

        pose = (crazyflieInfo[x].position)
        goal = (taskToRank.position)
        dist[x] = sqrt(np.sum((pose-taskToRank.position)**2, axis = 0))
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

    dtype = [('drone','S10'), ('distance', float), ('rankValue',float)]
    drones = [('alpha', dist[0], overallRank0), ('beta',dist[1], overallRank1), ('charlie',dist[2], overallRank2), ('delta',dist[3], overallRank3), ('echo',dist[4], overallRank4)]
    pre_ranked = np.array(drones, dtype=dtype)

    rank = np.sort(pre_ranked, order='rankValue') #highest rank is rank1 or rank[0]
    return rank


def task_alloc(rank,taskToAlloc):
    #input: rank vector each crazyflie, task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task

    print("start assignment")
    print(rank) 
    for x in range (0,5):  #check if first ranked drone is free, and assigns task
        for y in range (0,5):
            #print(crazyflieInfo[y].name)
            #print(rank[x][0])
            if(crazyflieInfo[y].name == rank[x][0]):
                #print("found drone name")
                if(crazyflieInfo[y].taskAssigned.name == 'not assigned'):
                    crazyflieInfo[y].assignTask(taskToAlloc, rank[x][2])
                    print(taskToAlloc.name)
                    print("task assigned successfully")
                    return
                else: #if the drones current task has a rank value of 1 less than the new task, it switches tasks ----- switching policy here
                    if(crazyflieInfo[y].taskRank + 1 < rank[x][2]):
                        oldTaskToReassign = crazyflieInfo[y].taskAssigned
                        crazyflieInfo[y].assignTask(taskToAlloc, rank[x][2])
                        #line below calls the task allocation for the task that was just taken from the other drone
                        task_alloc(mrta_rank(oldTaskToReassign),oldTaskToReassign)



#to be used to compare as a more basic assignment versus the more complex version above
def domino_assign(taskToAlloc):
    #input: task to allocate class object
    #output: assigns task name to taskAssigned of robot to do task

    #goes through drones in order until a drone is found to do the task
    for x in range (0,5):
        if(crazyflieInfo[x].taskAssigned == 'no Task'):
            crazyflieInfo[x].assignTask(taskToAlloc)

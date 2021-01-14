import numpy as np
import random
from math import pi, cos, sin, atan, atan2, sqrt, acos

import allocTask.py

height = 1 #height for all drones that will be ussed, here to make it easier to change if need be

seed = 0 #used to set the seed for task order assignment, if set to 1 will default to a preset order, anything else is random

robotParamOne = 1
robotParamTwo = 2
robotParamThree = 3     #to easily change what we define as the parameters

assignOrder = [0,1,2,3,4,5,6,7,8,9,10]  #order that the tasks will be assigned, changed later but default is this
numTasksAssigned = 0    #keeps track of the number of tasks assigned, and used to tell which task is assigned next

#creating the possible tasks to be assigned
possibleTasks = [
    taskZero = task('task zero', 1, 1, height, robotParamOne, 20),
    taskOne = task('task one', 3, 1, height, robotParamTwo, 10),
    taskTwo = task('task two', 2, 5, height, robotParamThree, 15),
    taskThree = task('task three', 3, 4, height, robotParamTwo, 25),
    taskFour = task('task four', 6, 9, height, robotParamOne, 5),
    taskFive = task('task five', 9, 3, height, robotParamTwo, 30),
    taskSix = task('task six', 6, 1, height, robotParamThree, 5),
    taskSeven = task('task seven', 7, 2, height, robotParamTwo, 30),
    taskEight = task('task eight', 2, 9, height, robotParamOne, 20),
    taskNine = task('task nine', 1, 6, height, robotParamTwo, 10),
    taskTen = task('task ten', 4, 5, height, robotParamThree, 15)
    ]
    
#list of tasks that have been assigned, is a list of 5
tasksAssigned = [task() for i in range(0,4)]


def task_generator()
    #input: none
    #output: returns the task that was just generated to be assigned/chosen to be assigned
    
    #Assigns one of the tasks that have not already been assigned in the possibleTasks array
    while(x!=5)
        for(x in range (0,4))
            if(taskAssigned(x).name == 'not assigned')
                taskAssigned(x) = random_task(seed, numTasksAssigned)
                numTasksAssigned = numTasksAssigned + 1
                x = 5
                return taskAssigned(x)

def random_task(seed, numTasksAssigned)
    #input: seed to possibly set order to preset order, the number of tasks that have already been assigned - starts at 0
    #output: the next task to be put forth for assignment
    
    #Decides the order in which tasks will be assigned and returns the next task to be put forth for assignment
    if(seed == 1)
        assignOrder = [0, 10, 1, 9, 2, 8, 3, 7, 4, 6, 5]
    else if(numTasksAssigned == 0)
        assignOrder = random.sample(range(11),11)
        
    return possibleTasks(numTasksAssigned)
    
    
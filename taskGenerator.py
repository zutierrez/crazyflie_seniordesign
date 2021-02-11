import numpy as np
import random
from math import pi, cos, sin, atan, atan2, sqrt, acos

import allocTask as taskk

height = 1 #height for all drones that will be ussed, here to make it easier to change if need be

seed = 0 #used to set the seed for task order assignment, if set to 1 will default to a preset order, anything else is random

robotParamOne = 1
robotParamTwo = 2
robotParamThree = 3     #to easily change what we define as the parameters

assignOrder = [0,1,2,3,4,5,6,7,8,9,10]  #order that the tasks will be assigned, changed later but default is this
numTasksAssigned = 0    #keeps track of the number of tasks assigned, and used to tell which task is assigned next

#creating the possible tasks to be assigned
possibleTasks = [
    taskk.task('task zero', 1, 1, height, robotParamOne, 20),
    taskk.task('task one', 3, 1, height, robotParamTwo, 10),
    taskk.task('task two', 2, 5, height, robotParamThree, 15),
    taskk.task('task three', 3, 4, height, robotParamTwo, 25),
    taskk.task('task four', 6, 9, height, robotParamOne, 5),
    taskk.task('task five', 9, 3, height, robotParamTwo, 30),
    taskk.task('task six', 6, 1, height, robotParamThree, 5),
    taskk.task('task seven', 7, 2, height, robotParamTwo, 30),
    taskk.task('task eight', 2, 9, height, robotParamOne, 20),
    taskk.task('task nine', 1, 6, height, robotParamTwo, 10),
    taskk.task('task ten', 4, 5, height, robotParamThree, 15)
    ]

#task to be assigned once all other tasks are finished    
goHome = taskk.task('task home', 0, 0, height, 0, 0)

#list of tasks that have been assigned, is a list of 5
tasksAssigned = [taskk.task() for i in range(0,4)]


def init_task_generator():
    #input: none
    #output: none

    #Assigns the first five tasks to be completed by the drones
    numTasksAssigned = 0

    for x in range (0,4):
        if(tasksAssigned[x].name == 'not assigned'):
            tasksAssigned[x] = random_task(seed, numTasksAssigned)
            numTasksAssigned = numTasksAssigned + 1

def new_task_generator():
    #input: none
    #output: the new task assigned to the drone

    #assigns a new task to the list of tasks once a task is complete

    if(numTasksAssigned == 11):
	taskToAssign = goHome
    else:
	taskToAssign = random_task(seed, numTasksAssigned)
	numTasksAssigned = numTasksAssigned + 1

    return taskToAssign

def random_task(seed, numTasksAssigned):
    #input: seed to possibly set order to preset order, the number of tasks that have already been assigned - starts at 0
    #output: the next task to be put forth for assignment
    
    #Decides the order in which tasks will be assigned and returns the next task to be put forth for assignment
    if(seed == 1):
        assignOrder = [0, 10, 1, 9, 2, 8, 3, 7, 4, 6, 5]
    elif(numTasksAssigned == 0):
        assignOrder = random.sample(range(11),11)
        
    nextTask = possibleTasks[numTasksAssigned]
    return nextTask
    
    

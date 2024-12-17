import math
import random
import numpy as np
import actually_generate
import Partition_Algorithm
import Graph_RSP_ILP
 #Build sensible path connecting all tasks, compare partition on that path vs ILP

MIN_DURATION=3

def build_path(V,E,tasks,robots):
    print(V,E)
    PA_input = []
    instance_to_input = []
    indices = {}
    y_coord= 0
    task_locations = [task[0] for task in tasks]
    new_robots = []
    for i in range(int(len(V)/2)):#  len/2 is m for a 2xm grid
        if (i,y_coord) in task_locations:
            PA_input.append(tasks[task_locations.index((i,y_coord))][1])
            instance_to_input.append((i,y_coord))
            print(PA_input)
        elif (i,y_coord) in robots:
            new_robots.append(len(PA_input))
            PA_input.append(0)
            instance_to_input.append((i,y_coord))

        if (i,1-y_coord) in task_locations :
            if (i,y_coord) not in instance_to_input: #i.e. no task or robot at (i,ycoord)
                PA_input.append(0)
                instance_to_input.append((i,y_coord))
                
            PA_input.append(tasks[task_locations.index((i,1-y_coord))][1])
            y_coord = 1- y_coord
            instance_to_input.append((i,y_coord))
        elif (i,1-y_coord) in robots:
            if (i,y_coord) not in instance_to_input: #i.e. no task or robot at (i,ycoord)
                PA_input.append(0)
                instance_to_input.append((i,y_coord))
            y_coord = 1- y_coord
            new_robots.append(len(PA_input))
            PA_input.append(0)
            instance_to_input.append((i,y_coord))
    return PA_input,new_robots, instance_to_input


for n in range(2,20):
    V= [(i,j) for i in range(n) for j in [0,1] ]
    #V= [i for i in range(2*n) ]
    E = [((i,0),(i,1)) for i in range(n)]
    E.extend([((i,j),(i+1,j)) for i in range((int(len(V)/2))-1) for j in range(2)])
    for duration_total in range(MIN_DURATION,3*n):
        for k in range(2,n):
            instances = list(actually_generate.generate_instances(2*(n-1),duration_total) )
            for i in range(1000):
                if len(instances) == 0:
                    break
                instance_index = random.randint(0,len(instances)-1)
                instance = instances[instance_index]
                temp = instances.copy()
                instances = instances[:instance_index]
                instances.extend(temp[instance_index+1:])
                task_locations = list(np.nonzero(instance)[0])
                tasks=[(task,int(instance[task])) for task in task_locations]
                print(tasks)
                tasks=[((math.floor(task[0]/2), task[0]%2 ), task[1]) for task in tasks]
                print(tasks)
                robots = []
                for i in range(k):
                    randX = random.randint(0,n-1)
                    randY = random.randint(0,1)
                    robots.append((randX,randY))
                
                path= build_path(V,E,tasks,robots)
                print(path)

                IP = Graph_RSP_ILP.Optimize_Robot_Scheduling(V,E,tasks,robots)

#print(build_path(V,E, [((0,1),3),((1,1),2),((2,0),1),((3,1),1),((4,0),3)],[(0,0),(2,0),(5,1)]) )
         
         



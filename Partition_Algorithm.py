import numpy as np
import networkx as nx
import math


def C_1(graph,location_of_robot):
    task_locations = list(np.nonzero(graph)[0])
    if len(task_locations) == 0:
        return 0
    leftmost_task = task_locations[0]
    rightmost_task = task_locations[-1]
    left_first = sum([graph[i] for i in task_locations]) + (rightmost_task - leftmost_task + abs(location_of_robot - leftmost_task ))
    right_first = sum([graph[i] for i in task_locations]) + (rightmost_task - leftmost_task + abs(location_of_robot - rightmost_task))
    return min(left_first,right_first)
def Partition_Algorithm(graph,locations_of_robots):
    task_locations = list(np.nonzero(graph)[0])
    S = np.zeros(shape=(len(locations_of_robots), len(task_locations)))
    split = np.zeros(shape=(len(locations_of_robots), len(task_locations)))
    split_point = 0
    for l in range(len(task_locations) ):
        S[0][l] = C_1(graph[:task_locations[l]+1],locations_of_robots[0])
#    for c in range(1,len(locations_of_robots)):
#        S[c][0] = min([C_1(graph[:max(locations_of_robots[i],task_locations[0])+1], locations_of_robots[i]) for i in range(c)])
    for c in range(1,len(locations_of_robots)):
        for l in range(len(task_locations) ):
            current_min = float('inf')
            for r in range(l+1):
                temp_graph = graph.copy()
                for i in range(task_locations[r]+1):
                    temp_graph[i]=0
                for i in range(task_locations[l]+1,len(graph)):
                    temp_graph[i]=0
                #temp_graph=graph[task_locations[r]+1:task_locations[l]+1]
                current_val=max(S[c-1][r],C_1(temp_graph,locations_of_robots[c]))
#                print("TEMP= ",temp_graph, f"current_val={current_val}, c= {c} , l = {l}")
                if current_val < current_min :
                    current_min = current_val
                    split_point = r 
            split[c][l]=split_point
            S[c][l] = current_min
    print(S)
    print(split)
    return int(S[len(locations_of_robots)-1][len(task_locations)-1])



#def Partition_Algorithm(graph,locations_of_robots):
#    task_locations = list(np.nonzero(graph)[0])
#    current_min=(math.inf,-1)
#    if len(task_locations)==1:
#        if task_locations[0] == 0 :#left has to do it
#            current_min = (C_1(graph,locations_of_robots[0]),0)
#        elif task_locations[0] == number_of_nodes-1 :#right has to do it
#            current_min = (C_1(graph,locations_of_robots[1]),number_of_nodes-1)
#        else:
#            split_point=task_locations[0]
#            left_graph = [graph[i]  for i in range(split_point+1)]
#            left_graph.extend([0 for i in range(split_point+1,number_of_nodes)])
#            right_graph = [0 for i in range(split_point+1)]#.extend([graph[i]  for i in range(split_point+1,number_of_nodes)])
#            right_graph.extend([graph[i]  for i in range(split_point+1,number_of_nodes)])
#            left_schedule = C_1(left_graph, locations_of_robots[0]) 
#            right_schedule = C_1(right_graph,locations_of_robots[1]) 
#            maxLR = max(left_schedule,right_schedule)
#            if maxLR != math.inf and maxLR < current_min[0]:
#                current_min = (maxLR,split_point)
#    else:
#        for split_point in task_locations:
#            left_graph = [graph[i]  for i in range(split_point+1)]
#            left_graph.extend([0 for i in range(split_point+1,number_of_nodes)])
#            right_graph = [0 for i in range(split_point+1)]#.extend([graph[i]  for i in range(split_point+1,number_of_nodes)])
#            right_graph.extend([graph[i]  for i in range(split_point+1,number_of_nodes)])

#            left_schedule = C_1(left_graph, locations_of_robots[0]) 
#            right_schedule = C_1(right_graph,locations_of_robots[1]) 
#            maxLR = max(left_schedule,right_schedule)
#            if maxLR != math.inf and maxLR < current_min[0]:
#                current_min = (maxLR,split_point)
#    return current_min

if __name__=="__main__":
    print(Partition_Algorithm([0,0,1,1, 1,1,  1] , [1, 2,5,6]))
    5
    
    #print(Partition_Algorithm([1,1,4,1,1,0],[2,5]))
    

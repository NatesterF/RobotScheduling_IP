import numpy as np
import time
import Partition_Algorithm
import robot_scheduling_ILP
import itertools
MIN_SIZE=15
MAX_SIZE = 20
MIN_DURATION = 3
MAX_DURATION = 20
MAX_ROBOTS = 10


def remove_duplicates(theList):
    newList = []
    for item in theList:
        if not item in newList:
            newList.append(item)
    return(newList)
def adds_up_to_n(n):
    theList= [[n]]
    if n <= 1:
        return [[n]]
    else:
        for sum_list in adds_up_to_n(n-1):
            temp = sum_list.copy()
            temp.extend([1])
            theList.append(temp)
            for i in range(len(sum_list)):
                temp = sum_list.copy()
                temp[i]+=1
                theList.append(temp)
    return remove_duplicates(theList)

def generate_instances(n,sum_of_durations):
    instances = []
    task_duration_list = adds_up_to_n(sum_of_durations)
    for tasks in task_duration_list:
        m = len(tasks)
        if m > n:
            break #can't have more tasks than vertices  
        for task_placements in itertools.combinations(range(n),m):
            instance= np.zeros(n)
            for i in range(len(task_placements)):
                instance[task_placements[i]] = tasks[i]
            task_list = [(task_placements[i],tasks[i]) for i in range(len(tasks))]
            instances.append(instance)
    return instances

if __name__ == "__main__":
    f = open("output.csv","w") 
    f.write("n,k,tasks,robot_locations, Partition Algorithm timespan, Optimal timespan,PA runtime, Gurobi runtime\n")
    f.close()

    count =0 

    n=MIN_SIZE
    while n <= MAX_DURATION:
        if count<=1200:
            for sum_of_durations in range(MIN_DURATION,MAX_DURATION):
                for instance in generate_instances(n,sum_of_durations):
                    for k in range(2,min(MAX_ROBOTS,n)):
                        for locations_of_robots in itertools.combinations(range(len(instance)),k):
                            if count>=(1200*50):
                                n+=1
                                count=0
                                break
                            if count % 50 == 0:
                                task_locations = list(np.nonzero(instance)[0])
                                tasks=[(task,int(instance[task])) for task in task_locations]
                                init_time = time.time()         
                                pa = Partition_Algorithm.Partition_Algorithm(instance,locations_of_robots)
                                after_pa_time = time.time()
                                ip = robot_scheduling_ILP.Optimize_Robot_Scheduling(len(instance),tasks,locations_of_robots)
                                after_ip_time =time.time()
                                print(instance, tasks,locations_of_robots,"n=",len(instance),"k=",k,"pa=",pa,"time=",patime:=(after_pa_time-init_time), ",ip=",ip,"time=",iptime:=(after_ip_time-after_pa_time),"TIME RATIO: ",iptime/patime ) 
                                f = open("output.csv","a") 
                                f.write(f"{n},{k},\"{tasks}\",\"{locations_of_robots}\", {pa} ,{ip},{patime},{iptime}\n")
                                f.close()
                                
                                print("######################",count)
                                if pa < ip:
                                    print("\033[91mSomething Went Wrong\033[0m ")
                            count+=1
            else:
                n+=1

                count =0 
                                                                                                                                                                                                                                        #    for n in range(MIN_SIZE,MAX_SIZE):
                                                                                                                                                                                                                                       #        for sum_of_durations in range(MIN_DURATION,2*n):
#            for instance in generate_instances(n,sum_of_durations):
#                for k in range(2,min(MAX_ROBOTS,n)):
#                    for locations_of_robots in itertools.combinations(range(len(instance)),k):
#                        task_locations = list(np.nonzero(instance)[0])
#                        tasks=[(task,int(instance[task])) for task in task_locations]
#                        init_time = time.time()         
#                        pa = Partition_Algorithm.Partition_Algorithm(instance,locations_of_robots)
#                        after_pa_time = time.time()
#                        ip = robot_scheduling_ILP.Optimize_Robot_Scheduling(len(instance),tasks,locations_of_robots)
#                        after_ip_time =time.time()
#                        print(instance, tasks,locations_of_robots,"n=",len(instance),"k=",k,"pa=",pa,"time=",patime:=(after_pa_time-init_time), ",ip=",ip,"time=",iptime:=(after_ip_time-after_pa_time),"TIME RATIO: ",iptime/patime ) 
#                        f = open("output.csv","a") 
#                        f.write(f"{n},{k},\"{tasks}\",\"{locations_of_robots}\", {pa} ,{ip},{patime},{iptime}\n")
#                        f.close()
#                        
#                        if pa < ip:
#                            print("\033[91mSomething Went Wrong\033[0m ")
#
#
                        


                

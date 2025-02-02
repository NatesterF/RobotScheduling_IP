import time
import random
import sys
import Partition_Algorithm 
import Graph_RSP_ILP
MIN_DURATION=3
MAX_DURATION=12
NUM_OF_INSTANCES=500
def grid_edges(n,m):
    V=[i+1 for i in range(n*m)]
    for i in range(n-1):
        for j in range(m-1):
            yield(i+(n*j),i+(n*j)+1) #horiz edges
            yield(i+(n*j),i+(n*(j+1)))
    for i in range(n-1):
        yield(i+((m-1)*n),i+(n*(m-1)+1)) #horiz edges
    for j in range(m-1):
            yield(n+(n*j)-1,n+(n*(j+1)-1))



def flatten_grid(n,m,tasks,robots,direction=1):
    if direction ==1:
        cur = 0
    else:
        cur = n-1
    task_locations = [task[0] for task in tasks]
    task_durations = [task[1] for task in tasks]
    line = []
    new_robots=[]
    while cur < n*m:
        print(cur)
        if cur in robots:
            new_robots.append(len(line))
        if cur in task_locations:
            line.append(task_durations[task_locations.index(cur)])
        else:
            line.append(0)
        if direction == 1 and cur % n == n-1:
            cur += n
            direction = -1
        elif direction == -1 and cur % n == 0:
            cur += n
            direction = 1
        else:
            cur+= direction
    print(line)
    print(new_robots)
    return Partition_Algorithm.Partition_Algorithm(line,new_robots)
            
if __name__ == "__main__":
    if len(sys.argv) != 3:
        
    #    m=3
    #    n=3
    #    robots=[2,3]
#   #     tasks=[(1,1),(2,3),(3,1),(12,2),(31,20)]
    #    tasks=[(1,1),(2,3),(3,1)]
#   #     print(minimal_ham_path(n,m,tasks,robots))
    #    print(list(grid_edges(n,m)))
#        print(Graph_RSP_ILP.Optimize_Robot_Scheduling(list(range(1,m*n+1)),list(grid_edges(n,m)),tasks,robots,1))

        for m in range(4,6):
            for n in range(2,6):
                for duration in range(MIN_DURATION,MAX_DURATION+1):
                    for k in range(2,min(6,n*m-1)):
                        seen=[]
                        while len(seen) <= NUM_OF_INSTANCES:
                            robots=[]
                            tasks = []
                            for i in range(k):
                                location = random.randrange(0,n*m)
                                while location in robots:
                                    location = random.randrange(0,n*m)
                                robots.append(location)
                            temp = duration
                            while temp > 0:
                                task_duration = random.randint(1,temp)
                                location = random.randrange(0,n*m)
                                locations = [task[0] for task in tasks]
                                if location in locations :
                                    tasks[locations.index(location)] = (location,tasks[locations.index(location)][1] + task_duration)
                                else:
                                    tasks.append((location,task_duration))
                                temp -= task_duration
                            if (robots,tasks) not in seen:
                                seen.append((robots,tasks))
                                print(len(seen))
                                print(f"n={n},m={m},tasks={tasks},robs={robots}")

                                init_time = time.time()         
                                pa =flatten_grid(n,m,tasks,robots) 
                                pa = min(pa,flatten_grid(n,m,tasks,robots,-1))
                                after_pa_time = time.time()
                                ip=Graph_RSP_ILP.Optimize_Robot_Scheduling(list(range(m*n)),list(grid_edges(n,m)),tasks,robots)
                                after_ip_time =time.time()
                                print(ip,pa)
                                f=open("grid_comparison.csv","a")
                                f.write(f"{m},{n},{k},{len(tasks)},{duration},{tasks},{robots},{pa},{ip},{after_pa_time-init_time},{after_ip_time-after_pa_time}\n")

                   
    #else:
    #    E= [edge for edge in grid_edges(sys.argv[1],sys.argv[2])]
    #    print(E)

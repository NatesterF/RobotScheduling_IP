import networkx as nx
import math
maths=math
import matplotlib.pyplot as plt
import gurobipy  as gp
from gurobipy import GRB
import numpy as np
from functools import * 
import itertools
import operator
import sys
def neighbourhood(v,E): #Generates all vertices adjacent to v
    for (v_1,v_2) in E:
        if v == v_1:
            yield v_2
        elif v == v_2:
            yield v_1


def binary_tree(n):
    for i in range(n):
        if 2*i + 1 < n:
            yield(i, 2*i+1)
        if 2 * i + 2 < n:
            yield(i, 2*i+2)
    
def grid_edges(n,m):
    V=[i+1 for i in range(n*m)]
    for i in range(1,n):
        for j in range(m-1):
            yield(i+(n*j),i+(n*j)+1) #horiz edges
            yield(i+(n*j),i+(n*(j+1)))
    for i in range(1,n):
        yield(i+((m-1)*n),i+(n*(m-1))+1) #horiz edges
    for j in range(m-1):
            yield(n+(n*j),n+(n*(j+1)))

def Optimize_Robot_Scheduling(V,E,tasks,sv, verbose=0):
    n=len(V)
    LIFETIME=(n)*len(sv) + (reduce(operator.add, [tasks[i][1] for i in range(len(tasks))],0))
#    tasks=[(task[0],task[1]+1) for task in tasks]
        # Create a new model
    m = gp . Model ( "RobotScheduling" )
    # Create variables
    x=m.addVars(len(sv),n,LIFETIME,vtype=GRB.BINARY,name="x") # x[r,v,t] = true if robot i at vertex v at the end of timestep t
    mo=m.addVars(len(sv),n,LIFETIME,vtype=GRB.BINARY,name="mo") # mo[r,v,t] = 1 if robot r just moved to vertex v during timestep t
    TCR = m.addVars(len(sv),len(tasks),LIFETIME,vtype=GRB.BINARY, name="TCR") #TCR[r,i,t] = task i complete by robot r at time t
    TC = m.addVars(len(tasks),LIFETIME,vtype=GRB.BINARY, name="TC") # TC[i,t] = task i complete at timestep t
    AC= m.addVars(LIFETIME,vtype=GRB.BINARY,name="AC") #ALL_COMPLETE at time t
    TS= m.addVar(vtype=GRB.INTEGER,name="TS") #ALL_COMPLETE but this time its an integer
    for i in range(len(sv)):
        m.addConstr(x[i,sv[i],0] == 1, name="initialise x sv_r")
        m.addConstr(mo[i,sv[i],0] == 1, name="initialise mo sv_r")
    
    for r in range(len(sv)):
        for i in range(len(tasks)):
            for t in range(tasks[i][1]):
                m.addConstr(TCR[r,i,t] == 0 ,name="TCR_Initial_Values") #there's no way a task can be completed before t= its duration
    
    for r in range(len(sv)):
        m.addConstrs((TCR[r,i,t] <=  TCR[r,i,t-1] + (gp.quicksum((1-mo[r,tasks[i][0],j])*x[r,tasks[i][0],j] for j in range(t - tasks[i][1],t))/(tasks[i][1]))  for i in range(len(tasks)) for t in range(tasks[i][1],LIFETIME) ) ,name=f"task_complete_R{r}")
#        m.addConstrs((TCR[r,i,t] <=  TCR[r,i,t-1] + (gp.quicksum(x[r,tasks[i][0],j] for j in range(t - tasks[i][1],t))/(tasks[i][1]))  for i in range(len(tasks)) for t in range(tasks[i][1],LIFETIME) ) ,name=f"task_complete_R{r}")
    
    
    m.addConstrs((TC[i,t] >= TC[i,t-1] for i in range(len(tasks)) for t in range(1,LIFETIME)),name="TC_stays_complete")
    m.addConstrs((TCR[r,i,t] >= TCR[r,i,t-1] for r in range(len(sv)) for i in range(len(tasks)) for t in range(1,LIFETIME)),name="TCR_stays_complete")
    
    
    m.addConstrs((TC[i,t]  == gp.quicksum(TCR[r,i,t]  for r in range(len(sv))) for i in range(len(tasks)) for t in range(LIFETIME) ),name="TC iff TCR")
    
    
    for r in range(len(sv)):
        m.addConstrs((gp.quicksum(x[r,v,t]  for v in range(n))==1 for t in range(LIFETIME)),name=f"R_{r} only_one_place")
        m.addConstrs((gp.quicksum(mo[r,v,t]  for v in range(n))<=1 for t in range(LIFETIME)))
        

    #m.addConstrs((mo[r,v,t] == gp.quicksum(x[r,v_prime,t-1] * x[r,v,t] for v_prime in [v-1,v+1] ) for r in range(len(sv)) for v in range(1,n-1) for t in range(1,LIFETIME)),name="just_moved")
    m.addConstrs((mo[r,v,t] == gp.quicksum(x[r,v_prime,t-1] * x[r,v,t] for v_prime in neighbourhood(v,E) ) for r in range(len(sv)) for v in range(n) for t in range(1,LIFETIME)),name="just_moved")
    #m.addConstrs((mo[r,0,t] == x[r,1,t-1] * x[r,0,t]  for r in range(len(sv))  for t in range(1,LIFETIME)),name="just_moved")
    #m.addConstrs((mo[r,n-1,t] == x[r,n-2,t-1] * x[r,n-1,t]  for r in range(len(sv))  for t in range(1,LIFETIME)),name="just_moved")
    m.addConstrs((gp.quicksum(x[r,v,t] for r in range(len(sv)))<=1 for v in range(n) for t in range(LIFETIME)), name="collision_free")
   # m.addConstrs(x[r,v,t] + x[r,v_prime,t+1] + x[r_prime,v_prime,t]+x[r_prime,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for (v,v_prime) in E for t in range(LIFETIME-1))
    m.addConstrs(x[r_prime,v,t] + x[r_prime,v_prime,t+1] + x[r,v_prime,t]+x[r,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for (v,v_prime) in E for t in range(LIFETIME-1))
    #m.addConstrs(x[r,v,t] + x[r,v+1,t+1] + x[r_prime,v+1,t]+x[r_prime,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for v in range(n-1) for t in range(LIFETIME-1))
    #m.addConstrs(x[r_prime,v,t] + x[r_prime,v+1,t+1] + x[r,v+1,t]+x[r,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for v in range(n-1) for t in range(LIFETIME-1))

    #m.addConstrs((x[r,v,t]<= (x[r,v-1,t-1]+x[r,v,t-1]+x[r,v+1,t-1])  for v in range(1,n-1) for t in range(1,LIFETIME) for r in range(len(sv))),name="Adjacency") #Adjacency conditions
    m.addConstrs((x[r,v,t]<= ( x[r,v,t-1]+gp.quicksum(x[r,v_prime,t-1] for v_prime in neighbourhood(v,E)))  for v in range(n) for t in range(1,LIFETIME) for r in range(len(sv))),name="Adjacency") #Adjacency conditions
    #m.addConstrs((x[r,0,t]<= (x[r,0,t-1]+x[r,1,t-1])  for t in range(1,LIFETIME) for r in range(len(sv))),name="adjacency_first")
    #m.addConstrs((x[r,n-1,t]<= (x[r,n-2,t-1]+x[r,n-1,t-1]) for t in range(1,LIFETIME) for r in range(len(sv))),name="adjacency_last")
    
    m.addConstrs((AC[t] <= gp.quicksum(TC[i,t] for i in range(len(tasks)) )/len(tasks) for t in range(LIFETIME)),name="All_tasks_Complete?" )
    m.addConstr(gp.quicksum(AC[t] for t in range(LIFETIME)) == 1)
    
    m.addConstr(TS == gp.quicksum(t*(AC[t]) for t in range(LIFETIME)) )
    
    
    m.setObjective(TS -1  ,GRB.MINIMIZE)
        
    m.setParam('OutputFlag', 0)
    m.optimize()
    print("OPTMISZED")
    m.write("RobotScheduling.lp")

    if verbose and m.SolCount >=1 :
        for v in m . getVars ():
            print ( '%s %g ' % ( v . VarName , v . X ))
            print ( ' Obj : %g ' % m . ObjVal )
   
        schedule="{"
        for r in range(len(sv)):
            schedule+="\n["
            for t in range(int(TS.X) ):
                for v in range(n):
                    if x[r,v,t].X == 1:
                        schedule+=f"t{t}:{v},"
        
            schedule+="]"
        print(schedule,"}")
        return(int(TS.X -1 ), schedule)
    else:
        return(float("inf"))

if __name__=="__main__":

    if len(sys.argv) == 1:
    #    input_tasks=[(1,3),(3,2),(4,5)]
        #input_tasks =  [(3, 2)]
#        E=[(0,1),(0,3), (1,3),(1,4),(1,2),(3,4)]
#        sv=[6,8]
        #f=open("test.txt","a")
    #for n in range(14,17):
    #        V=[i for i in range(n)]
    #        E=[(e_1,e_2) for (e_1,e_2) in binary_tree(n)]   
    #        leaves=[i for i in range(list(neighbourhood(n-1,E))[0]+1,n)]
    #        first_robot=2**(maths.floor((math.log(n+1,2))))-1
    #        #E=[(e_1,e_2) for (e_1,e_2) in binary_tree(n)]   
#   #         sv=[7,14]

    #        sv=[leaves[-1],leaves[-2]]
    #        input_tasks=[(leaves[i],1) for i in range(len(leaves))]

    #        print(Optimize_Robot_Scheduling(V,E,input_tasks,sv,1))
    #        if first_robot not in sv:
    #            print(Optimize_Robot_Scheduling(V,E,input_tasks,[first_robot,leaves[-1]]))
    #        else:
    #            print(Optimize_Robot_Scheduling(V,E,input_tasks,[first_robot-1,leaves[-1]]))
    #                
            
        G = nx.Graph()
        G.add_edges_from(E)
        nx.draw(G,with_labels=True)
        print(input_tasks,sv)
        plt.show()
        print(E)
    elif len(sys.argv) != 6:
        print("USAGE: python robot_scheduling_ILP.py <vertices> <edges>  <task locations> <task durations> <robot locations>")
        
    else:
        n=int(sys.argv[1])
                                                                 
        input_tasks_locations=sys.argv[2][1:len(sys.argv[2])-1]
        input_tasks_durations=sys.argv[3][1:len(sys.argv[3])-1]
        
        input_tasks_locations = input_tasks_locations.split(',')
        input_tasks_durations = input_tasks_durations.split(',')
        input_tasks = [(int(input_tasks_locations[i])-1,int(input_tasks_durations[i])) for i in range(len(input_tasks_locations))]
        sv= sys.argv[4][1:len(sys.argv[4])-1]
        sv=sv.split(',')
        sv=[int(vertex)-1 for vertex in sv]
        print(Optimize_Robot_Scheduling(n,input_tasks,sv,1))
     

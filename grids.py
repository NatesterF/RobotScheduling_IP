import sys
import Partition_Algorithm 
import Graph_RSP_ILP
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



def minimal_ham_path(n,m,tasks,robots=[]):
    current_min = float('inf')
    i=2 
    row = 0
    previ= 1
    graph=[  tasks[0][1] if (tasks[0][0] == 1)  else 0]#input to partition algorithm
    graph_labelling=[1]# used to turn the grid vertices into the path vertices
    E = []
    while i <= n*m :
        E.append((previ,i))
        if i in [task[0] for task in tasks]: #if there is a task at vertex i  
            graph.append([task[1] for task in tasks if task[0] == i ][0])
        else:
            graph.append(0)
        graph_labelling.append(i)
        if i % m == 0 and previ % m != 0: #end of row
            previ=i
            i+=m
            row += 1
        elif i % m == 1 and previ % m == 2: #end of row
            previ=i
            i+=m
            row += 1
        else:
            previ=i
            i+=((-1)**row)

      
    print(graph)
    print(graph_labelling)
    robots_on_new_graph = [i for i in range(len(graph_labelling)) if graph_labelling[i] in robots ]
    print(robots_on_new_graph)
    
    return Partition_Algorithm.Partition_Algorithm(graph,robots_on_new_graph)



if __name__ == "__main__":
    if len(sys.argv) != 3:
        
        m=6
        n=6
        robots=[2,20,10]
        tasks=[(1,1),(2,3),(3,1),(12,2),(31,20)]
        print(minimal_ham_path(6,6,tasks,robots))
        print(Graph_RSP_ILP.Optimize_Robot_Scheduling(range(1,m*n+1),grid_edges(m,n),tasks,robots))
    else:
        E= [edge for edge in grid_edges(sys.argv[1],sys.argv[2])]
        print(E)

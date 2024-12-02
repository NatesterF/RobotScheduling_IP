import csv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})


f = open('output.csv',newline='') 
reader = csv.reader(f,delimiter=",", quotechar='\"')
#n,k,tasks,robot_locations, Partition Algorithm timespan, Optimal timespan,PA runtime, Gurobi runtime
averagesPA=[] #used to keep track of averagesPA for each numbers of robots
averagesOPT=[] #used to keep track of averagesPA for each numbers of robots
for i in range(7):
    averagesPA.append({2:(0,0),3:(0,0),4:(0,0),5:(0,0),6:(0,0),7:(0,0)})
    averagesOPT.append({2:(0,0),3:(0,0),4:(0,0),5:(0,0),6:(0,0),7:(0,0)})
count = 0
for setting in reader:
    if count !=0:
        k=int(setting[1])
        task_list=eval(setting[2])
        m=len(task_list)
        averagesPA[m-1][k] = ((averagesPA[m-1][k][0]+float(setting[6]))/(averagesPA[m-1][k][1]+1),averagesPA[m-1][k][1]+1)
        averagesOPT[m-1][k] = ((averagesOPT[m-1][k][0]+float(setting[7]))/(averagesOPT[m-1][k][1]+1),averagesOPT[m-1][k][1]+1)

    count+=1 

print(averagesPA)
print(averagesOPT)


k=2

opt_times = [float(averagesOPT[m][k][0]) for m in range(7) ]
print(opt_times)
pa_times = [averagesPA[m][k][0] for m in range(7) ]
print(pa_times)

plt.plot(range(1,8),opt_times,label="Gurobi Runtime")
plt.plot(range(1,8),pa_times,label="Partition Algorithm Runtime")
plt.yscale('log')

plt.xlabel('number of tasks,m')
plt.ylabel('Runtime (in seconds)')
plt.title("Runtime comparison for k=2")

plt.legend()
#plt.savefig("runtime_k4.png")
plt.show()



import time
import os
import numpy as np
runningtime = time.time()
runtimelist = []
case = 0
Time = 0
while case < 25:
    runningtime = time.time()
    os.system('python main.py')
    print(Time)
    runtimelist.append(time.time() - runningtime)
    runningtime = time.time()
    print(runtimelist[-1])
    case += 1
runtimelist = np.array(runtimelist)
np.savetxt("triangle.csv", runtimelist, delimiter=",")
print(Time)

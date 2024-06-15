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
    print('TIME',runtimelist[-1])
    print('============================================')
    print('============================================')
    print('CASE', case)
    case += 1
    print('============================================')
    print('============================================')
runtimelist = np.array(runtimelist)
np.savetxt("smallangle.csv", runtimelist, delimiter=",")


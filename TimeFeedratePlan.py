import math
import numpy as np
from scipy.spatial.distance import sqeuclidean
from GCodeHandler import Movement
indexes = []
def planTime(times, spline, Movements: list, Amax, splineaxis, StartTime): #осейвой сплайн нуджен чтобы знать длину вектора времени после выкидывания точек
    T = []
    global indexes
    T = np.linspace(StartTime, 1 , len(splineaxis))
    print(len(spline), len(T))
    T[0] = StartTime
    Counter = 0 #счетчик движений
    Currenttime = 0
    End = False
    i = 0
    dt = math.sqrt(sqeuclidean([spline[0][0], spline[0][1], spline[0][2]], [spline[1][0], spline[1][1], spline[1][2]])) / ((Movements[0].speed))
    #print(spline)
    print('q1', len(splineaxis))
    while not End:
        if i < (len(splineaxis) - 1):
            if T[i] < (Currenttime + Movements[Counter].time):
                dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]], [spline[i+1][0], spline[i+1][1], spline[i+1][2]])) / (
                       Movements[Counter].speed)
                T[i + 1] = T[i] + dt
            #print('DT = ',dt)
            if T[i] == T[i + 1] or  dt == 0:
                print('dt = 0!', T[i], T[i + 1], dt)
                raise ValueError
            #print('T[i]', T[i], Currenttime + Movements[Counter].time, i, dt)
            if (T[i] >= (Currenttime + Movements[Counter].time)):
                if Counter < (len(Movements) - 1):
                    print('SPEED INFO', Movements[Counter + 1].speed)
                    Currenttime += Movements[Counter].time #abs((Movements[Counter].speed - Movements[Counter + 1].speed) / ((Amax)))
                    print(Currenttime, Counter, spline[i][0], spline[i][1], spline[i][2])
                    dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]],
                                     [spline[i + 1][0], spline[i + 1][1], spline[i + 1][2]])) / (
                                     Movements[Counter + 1].speed)
                    Counter += 1
                    indexes.append(i)
                    T[i + 1] = T[i] + dt
                    if i == (len(splineaxis) - 2):
                        i += 1
                    else:
                        i += 1
                    continue
                elif (Counter == (len(Movements) - 1)):
                    Currenttime += Movements[Counter].time
                    #print(Currenttime, Counter)
                    dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]],
                                     [spline[i + 1][0], spline[i + 1][1], spline[i + 1][2]])) / (
                                     Movements[Counter].speed)
                    T[i + 1] = T[i] + dt
                    if i == (len(splineaxis) - 2):
                        i += 1
                    else:
                        i += 2
                    continue
            i += 1
        else:
            End = True
            T = list(T)
            T.sort()
            #print(Movements)
            break
    print(type(T))
    return T




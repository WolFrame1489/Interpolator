import math
import numpy as np
from scipy.spatial.distance import sqeuclidean
from GCodeHandler import Movement
def planTime(times, spline, Movements: list, Amax, splineaxis): #осейвой сплайн нуджен чтобы знать длину вектора времени после выкидывания точек
    T = []
    T = np.arange(0, 1 , 1 / len((splineaxis)))
    print(len(spline), len(T))
    T[0] = 0.0
    Counter = 0 #счетчик движений
    Currenttime = 0
    End = False
    i = 2
    dt = math.sqrt(sqeuclidean([spline[0][0], spline[0][1], spline[0][2]], [spline[1][0], spline[1][1], spline[1][2]])) / ((Movements[0].speed))
    #print(spline)
    print('q1', len(splineaxis))
    while not End:
        if i < (len(splineaxis) - 1):
            dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]], [spline[i-1][0], spline[i-1][1], spline[i-1][2]])) / (
                       Movements[Counter].speed)
            T[i] = T[i - 1] + dt
            #print('T[i]', T[i], Currenttime + Movements[Counter].time, i, dt)
            if (T[i] >= (Currenttime + Movements[Counter].time)):
                if Counter < (len(Movements) - 1):
                    Currenttime += Movements[Counter].time #abs((Movements[Counter].speed - Movements[Counter + 1].speed) / ((Amax)))
                    #print(Currenttime, Counter, spline[i][0], spline[i][1], spline[i][2])
                    dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]],
                                     [spline[i + 1][0], spline[i + 1][1], spline[i + 1][2]])) / (
                                     Movements[Counter + 1].speed)
                    Counter += 1
                    T[i + 1] = Currenttime
                    if i == (len(splineaxis) - 2):
                        i += 1
                    else:
                        i += 2
                    continue
                elif (Counter == (len(Movements) - 1)):
                    Currenttime += Movements[Counter].time
                    #print(Currenttime, Counter)
                    dt = math.sqrt(sqeuclidean([spline[i][0], spline[i][1], spline[i][2]],
                                     [spline[i + 1][0], spline[i + 1][1], spline[i + 1][2]])) / (
                                     Movements[Counter].speed)
                    T[i + 1] = Currenttime
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




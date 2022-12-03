import math
import numpy as np
import scipy.interpolate
from geomdl import utilities
import GCodeHandler
from GCodeHandler import HandleGCode, weight, MoveList
from Splines import CreateNURBSCurve, PrepareBSpline, RebuildSpline, OptimizeNURBS
from Kins import ScaraInvKins,ScaraInvKins2, ScaraForwardKins
from scipy.interpolate import make_interp_spline, PPoly, splprep, UnivariateSpline
import os
import matplotlib.pyplot as plt
if __name__ == "__main__":
    x = []
    Jmax = 50
    Amax = 20
    Vmax = 1.5
    Vmove = 0.005
    GCodeHandler.weight = 1.0 # вес начальной точки
    realx = []
    y = []
    realy = []
    JointPoints = []
    CurrentPos = [150.0, 200.0, 0.0, 1] # начальная позиция робота
    filename = 'testtraj.cpt'
    gcodeFileName = 'square.txt'
    print('Linearizing...')
    os.system('python pygcode-norm.py  -al -alp 0.001 ' + gcodeFileName) #линеаризуем файл
    print('Reading G-code....')
    HandleGCode('coderework.txt', CurrentPos) # делаем точки из ж кода и выдаем им веса
    CartesianPoints = []
    deviation = 30
    print('Starting geomdl....')
    CartesianPoints = CreateNURBSCurve('testtraj.cpt', CurrentPos) # делаем нурбс интерполяцию в координатах мира
    print('geomdl finished...')
    print('optimizing Nurbs...')
    OptimizeNURBS(CartesianPoints)
    print('Optimization complete...')
    Limits = [[math.radians(-140), math.radians(140)], [math.radians(-160), math.radians(160)], [-100, 100]] # лимиты робота
    Vmaxq1 = 2.0
    Vmaxq2 = 1.7
    Vmaxq3 = 1.0
    JointPoints = ScaraInvKins(CartesianPoints, 175, 275, 100, Limits) # делаем ОЗК по полученным точкам
    x = []
    y = []
    q1 = []
    q2 = []
    q3 = []
    for i in range(len(JointPoints)):
        q1.append(JointPoints[i][0])
        q2.append(JointPoints[i][1])
        q3.append(JointPoints[i][2])
    # массивы координат каждой оси
    q1 = np.array(q1)
    q2 = np.array(q2)
    q3 = np.array(q3)
    T = 0.001 # время кадра системы
    T = np.arange(0, 10, 10/len(q1))
# создаем сплайны по 3 осям
    BSplines = PrepareBSpline(q1, q2, q3, T, 1, 0.0)
    knots = BSplines[0][0]
    Coefficients = []
    Coefficients.append(PPoly.from_spline(BSplines[0]).c)
    BSplines = PrepareBSpline(q1, q2, q3, T, 2, 0.0)
    Coefficients.append(PPoly.from_spline(BSplines[1]).c)
    BSplines = PrepareBSpline(q1, q2, q3, T, 3, 0.0)
    Coefficients.append(PPoly.from_spline(BSplines[2]).c)
    u = 3
    # скорости осей
    Vq1 = []
    Vq2 = []
    Vq3 = []
    # ускорения осей
    Aq3 = []
    Aq2 = []
    Aq1 = []
    # джиттер осей
    Jq1 = []
    Jq2 = []
    Jq3 = []
    # Контурные параметры
    Vx = []
    Vy = []
    Vz = []
    Ax = []
    Ay = []
    Az = []
    testx = []
    print('Creating polynome splines of speed etc...')
    while (u < (len(knots) - 3)):
       Vq1.append((5 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 4))+ (4 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 3)) \
                  + (3 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 2)) \
             + (2 * Coefficients[0][3][u] * ((knots[u] / len(knots)))) + (Coefficients[0][4][u]))
       Vq2.append((5 * Coefficients[1][0][u] * (knots[u] / len(knots)) ** 4) + (4 * Coefficients[1][1][u] * (knots[u] / len(knots)) ** 3) + (
                   3 * Coefficients[1][2][u] * (knots[u] / len(knots)) ** 2) \
                  + (2 * Coefficients[1][3][u] * (knots[u] / len(knots))) + (Coefficients[1][4][u]))
       Vq3.append((5 * Coefficients[2][0][u] * (knots[u] / len(knots)) ** 4) + (4 * Coefficients[2][1][u] * (knots[u] / len(knots)) ** 3) + (
               3 * Coefficients[2][2][u] * (knots[u] / len(knots)) ** 2) \
                  + (2 * Coefficients[2][3][u] * (knots[u] / len(knots))) + (Coefficients[2][4][u]))
       Aq1.append((20 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 3)) + (
                   12 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                              6 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 1)) \
                  + (2 * Coefficients[0][3][u]))
       Aq2.append((20 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 3)) + (
               12 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                          6 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 1)) \
                  + (2 * Coefficients[1][3][u]))
       Aq3.append((20 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 3)) + (
               12 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                          6 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 1)) \
                  + (2 * Coefficients[2][3][u]))
       Jq1.append((60 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 2)) + (
               24 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                          6 * Coefficients[0][2][u]))
       Jq2.append((60 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 2)) + (
               24 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                          6 * Coefficients[1][2][u]))
       Jq3.append((60 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 2)) + (
               24 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                          6 * Coefficients[2][2][u]))
       u += 1
    i = 1
    testV = Vq1
    testA = Aq1
    testJ = Jq1
    print('Starting spline fitting...')
    s = 0.0
    while i < (len(q1) - 3):
        print(i, s)
        #knots = utilities.generate_knot_vector(5, len(q1))
        if (abs(Jq1[i]) > Jmax):
            print('jitter1', Jq1[i], i)
            Jq1 = []
            Aq1 = []
            Vq1 = []
            #T = np.delete(T, i)
            #q1 = np.delete(q1, i)
            BSplines = PrepareBSpline(q1, q2, q3, T, 1, s)
            Coefficients[0] = PPoly.from_spline(BSplines[0]).c
            u = 3
            knots = BSplines[0][0]
            res = RebuildSpline(Vq1, Aq1, Jq1, Coefficients, knots, 1)
            Vq1 = res[0]
            Aq1 = res[1]
            Jq1 = res[2]
            i = 1
            print(len(knots))
        elif (abs(Aq1[i]) > Amax):
            print('accel1', Aq1[i], i)
            Jq1 = []
            Aq1 = []
            Vq1 = []

            #T = np.delete(T, i)
            #q1 = np.delete(q1, i)
            BSplines = PrepareBSpline(q1, q2, q3, T, 1, s)
            Coefficients[0] = PPoly.from_spline(BSplines[0]).c
            u = 3
            knots = BSplines[0][0]
            i = 1
            Vq1, Aq1, Jq1 = RebuildSpline(Vq1, Aq1, Jq1, Coefficients, knots, 1)
        else:
            i += 1
    T = np.arange(0, 10, 10 / len(q2))
    i = 1
    while i < (len(q2) - 3):
        print(i)
        #knots = utilities.generate_knot_vector(5, len(q2))
        if (abs(Jq2[i]) > Jmax):
            print('jitter2', Jq2[i], i)
            Jq2 = []
            Aq2 = []
            Vq2 = []

            #T = np.delete(T, i)
            #q2 = np.delete(q2, i)
            print(len(T), len(q2))
            BSplines = PrepareBSpline(q1, q2, q3, T, 2, s)
            Coefficients[1] = PPoly.from_spline(BSplines[1]).c
            u = 3
            knots = BSplines[1][0]
            res = RebuildSpline(Vq2, Aq2, Jq2, Coefficients, knots, 2)
            Vq2 = res[0]
            Aq2 = res[1]
            Jq2 = res[2]
        elif (abs(Aq2[i]) > Amax):
            print('accel2', Aq1[i], i)
            Jq2 = []
            Aq2 = []
            Vq2 = []
            #T = np.delete(T, i)
            #q2 = np.delete(q2, i)
            BSplines = PrepareBSpline(q1, q2, q3, T, knots,  2, s)
            Coefficients[1] = PPoly.from_spline(BSplines[1]).c
            u = 3
            knots = BSplines[1][0]
            Vq2, Aq2, Jq2 = RebuildSpline(Vq1, Aq1, Jq1, Coefficients, knots, 3)
        else:
            i += 1
    T = np.arange(0, 10, 10 / len(q3))
    i = 1
    while i < (len(q3) - 3):
        print(i)
       # knots = utilities.generate_knot_vector(5, len(q3))
        if (abs(Jq3[i]) > Jmax):
            print('jitter3', Jq3[i], i)
            Jq3 = []
            Aq3 = []
            Vq3 = []
            #T = np.delete(T, i)
            #q1 = np.delete(q3, i)
            BSplines = PrepareBSpline(q1, q2, q3, T, 3, s)
            Coefficients[2] = PPoly.from_spline(BSplines[0]).c
            u = 3
            knots = BSplines[2][0]
            res = RebuildSpline(Vq3, Aq3, Jq3, Coefficients, knots, 3)
            Vq3 = res[0]
            Aq3 = res[1]
            Jq3 = res[2]
            print(len(knots))
        elif (abs(Aq3[i]) > Amax):
            print('accel3', Aq1[i], i)
            Jq3 = []
            Aq3 = []
            Vq3 = []
            #T = np.delete(T, i)
            #q3 = np.delete(q1, i)
            BSplines = PrepareBSpline(q1, q2, q3, T, knots, 3, s)
            Coefficients[2] = PPoly.from_spline(BSplines[0]).c
            u = 3
            knots = BSplines[2][0]
            Vq3, Aq3, Jq3 = RebuildSpline(Vq3, Aq3, Jq3, Coefficients, knots)
        else:
            i += 1
realpoints = []
realpoints.append([])
realpoints.append([])
realpoints.append([])
realpoints[0].append(150)
realpoints[1].append(200)
realpoints[2].append(0)
for i in range((min(len(q1), len(q2), len(q3)))):
    realpoints[0].append((ScaraForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100))[0])
    realpoints[1].append((ScaraForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100))[1])
    realpoints[2].append((ScaraForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100))[2])



import plotly.graph_objects as go
fig = go.Figure(data=[go.Scatter(x=realpoints[0], y=realpoints[1])])
fig.show()
import plotly.express as px
from plotly.subplots import make_subplots
fig = go.Figure(data=[go.Scatter(x=T, y=Vq1), go.Scatter(x=np.arange(0, 10, 0.01), y=testV)])
fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Vq1), go.Scatter(x=np.arange(0, 10, 0.01), y=testA)])
fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Jq1), go.Scatter(x=np.arange(0, 10, 0.01), y=testJ)])
fig.show()
#test2 = scipy.interpolate.splev(T, BSplines[0])

#plt.plot(T, Jq1, 'b', label='q1')
#plt.plot(T, q2, 'r', label='q2')
#plt.plot(T, q3, 'g', label='q3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
print(len(Vq1))
T = np.arange(0, len(Vq1), 1)

#plt.plot(T, Vq1, 'b', label='Vq1')
#plt.plot(T, Vq2, 'r', label='Vq2')
#plt.plot(T, Vq3, 'g', label='Vq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
#plt.plot(T, Aq1, 'b', label='Aq1')
#plt.plot(T, Aq2, 'r', label='Aq2')
#plt.plot(T, Aq3, 'g', label='Aq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
#plt.plot(T, Jq1, 'b', label='Jq1')
#plt.plot(T, Jq2, 'r', label='Jq2')
#plt.plot(T, Jq3, 'g', label='Jq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()








import math
import numpy as np
import scipy.interpolate

import GCodeHandler
from GCodeHandler import HandleGCode, weight, MoveList
from Splines import CreateNURBSCurve, PrepareBSpline
from Kins import ScaraInvKins,ScaraInvKins2, ScaraForwardKins
from scipy.interpolate import make_interp_spline, PPoly
import os
import matplotlib.pyplot as plt
if __name__ == "__main__":
    x = []
    GCodeHandler.weight = 1.0 # вес начальной точки
    realx = []
    y = []
    realy = []
    JointPoints = []
    CurrentPos = [150.0, 200.0, 0.0, 1.0] # начальная позиция робота
    filename = 'testtraj.cpt'
    gcodeFileName = 'square.txt'
    os.system('python pygcode-norm.py  -al -alp 0.001 ' + gcodeFileName) #линеаризуем файл
    HandleGCode('coderework.txt', CurrentPos) # делаем точки из ж кода и выдаем им веса
    CartesianPoints = []
    deviation = 30
    CartesianPoints = CreateNURBSCurve('testtraj.cpt', CurrentPos) # делаем нурбс интерполяцию в координатах мира
    Limits = [[math.radians(-140), math.radians(140)], [math.radians(-160), math.radians(160)], [-100, 100]] # лимиты робота
    Vmaxq1 = 20000000
    Vmaxq2 = 17000000
    Vmaxq3 = 10000000
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
    T = 0.01 # время кадра системы
    T = np.arange(0, len(q1), 1)
# создаем сплайны по 3 осям
    BSplines = PrepareBSpline(q1, q2, q3)
    knots = BSplines[0][0]
    Coefficients = []
    Coefficients.append(PPoly.from_spline(BSplines[0]).c)
    Coefficients.append(PPoly.from_spline(BSplines[1]).c)
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









import plotly.graph_objects as go
import plotly.express as px

from plotly.subplots import make_subplots
fig = go.Figure(data=[go.Scatter(x=T, y=Vq1), go.Scatter(x=T, y=Vq2), go.Scatter(x=T, y=Vq3)])
#fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Aq1), go.Scatter(x=T, y=Aq2), go.Scatter(x=T, y=Aq3)])
#fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Jq1), go.Scatter(x=T, y=Jq2), go.Scatter(x=T, y=Jq3)])
#fig.show()
test2 = scipy.interpolate.splev(T, BSplines[0])

plt.plot(T, q1, 'b', label='q1')
plt.plot(T, q2, 'r', label='q2')
plt.plot(T, q3, 'g', label='q3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
print(len(Vq1))
plt.plot(T, Vq1, 'b', label='Vq1')
plt.plot(T, Vq2, 'r', label='Vq2')
plt.plot(T, Vq3, 'g', label='Vq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
plt.plot(T, Aq1, 'b', label='Aq1')
plt.plot(T, Aq2, 'r', label='Aq2')
plt.plot(T, Aq3, 'g', label='Aq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()
plt.plot(T, Jq1, 'b', label='Jq1')
plt.plot(T, Jq2, 'r', label='Jq2')
plt.plot(T, Jq3, 'g', label='Jq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
plt.show()








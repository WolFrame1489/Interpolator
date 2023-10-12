from derivative import dxdt
import math
import numpy as np
import scipy.interpolate
from geomdl import utilities
import GCodeHandler
from GCodeHandler import HandleGCode, weight, MoveList
from Splines import CreateNURBSCurve, PrepareBSpline, RebuildSpline, OptimizeNURBS
from TimeFeedratePlan import planTime
from Kins import InvKins,ScaraInvKins2, ForwardKins, ForwardSpeedKins
from scipy.interpolate import make_interp_spline, PPoly, BSpline, splprep, UnivariateSpline, spalde
import os
import csv
import matplotlib.pyplot as plt
if __name__ == "__main__":
    x = []
    DeltaRF = 1120.0
    DeltaRE = 2320.0
    DeltaF = 4570.0
    DeltaE = 1150.0
    Jmax = 600000000.5
    Amax = 600000000.5
    Vmax = 10.5
    Vmove = 0.005
    PosCycleTime = 0.000400 # время такта контура позиции
    GCodeHandler.weight = 1.0 # вес начальной точки
    realx = []
    y = []
    realy = []
    JointPoints = []
    CurrentPos = [0.1, 10.0, 0.1, 1] # начальная позиция робота
    Kinematics = 'TRIV'





    filename = 'testtraj.cpt'
    gcodeFileName = 'square.txt' #TODO: СЮДА ПИСАТЬ ИМЯ ФАЙЛА С G КОДОМ
    print('Linearizing...')
    print('getcwd:      ', os.getcwd())
    os.system('python pygcode-norm.py  -al -alp 0.1 -alm i  ' + (os.getcwd() + '\\' + gcodeFileName)) #линеаризуем файл
    print('Reading G-code....')
    HandleGCode('coderework.txt', CurrentPos, Vmax) # делаем точки из ж кода и выдаем им веса
    times = [] # список ля хранения времен, необходимых на каждое движение
    i = 0



    print(GCodeHandler.MoveList)
    while i < len(GCodeHandler.MoveList):
        print(GCodeHandler.MoveList[i].time)
        if GCodeHandler.MoveList[i].time != 0:
            times.append(GCodeHandler.MoveList[i].time)
        i+=1
    SumTime = sum(times)
    print(SumTime, times)
    PointsAmount = math.ceil((SumTime) * len(times) * 8000) # делаем грубое количество точек, чтобы потом решить сколько нам реально надо
    CartesianPoints = []
    deviation = 30




    print('Starting lirear interp....')
    CartesianPoints = CreateNURBSCurve('testtraj.cpt', CurrentPos, PointsAmount) # делаем линейную интерполяцию в координатах мира
    IdealpointsX = []
    IdealpointsY = []
    IdealpointsZ = []
    for i in range(len(CartesianPoints)):
        IdealpointsX.append(CartesianPoints[i][0])
        IdealpointsY.append(CartesianPoints[i][1])
        IdealpointsZ.append(CartesianPoints[i][2])
    import plotly.express as px




    fig = px.scatter(x=IdealpointsX, y=IdealpointsY, title="BSPLINETEST")
    fig.show()
    print('geomdl finished...')
    print('optimizing NURBS...')
    OptimizedPoints = OptimizeNURBS(CartesianPoints)
    print('Optimization complete...')
    Limits = [[math.radians(-140), math.radians(140)], [math.radians(-160), math.radians(160)], [-100, 100]] # лимиты робота
    Vmaxq1 = 2.0
    Vmaxq2 = 1.7
    Vmaxq3 = 1.0





    CartesianPoints = []
    for i in range(len(OptimizedPoints[0])):
        CartesianPoints.append([OptimizedPoints[0][i], OptimizedPoints[1][i], OptimizedPoints[2][i]])
    JointPoints = InvKins(CartesianPoints, 175, 275, 100, Limits, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF) # делаем ОЗК по полученным точкам
    x = []
    y = []
    q1 = []
    q2 = []
    q3 = []
    for i in range(len(JointPoints)):
        if i > 0 and JointPoints[i] != JointPoints[i - 1]:
            q1.append(JointPoints[i][0])
        if i > 0 and JointPoints[i] != JointPoints[i - 1]:
            q2.append(JointPoints[i][1])
        if i > 0 and JointPoints[i] != JointPoints[i - 1]:
            q3.append(JointPoints[i][2])





    # массивы координат каждой оси
    q1 = np.array(q1)
    q2 = np.array(q2)
    q3 = np.array(q3)
    # while (i < len(q1) - 1):
    #     if q1[i] == q1[i + 1]:
    #         q1 = np.delete(q1, i + 1)
    #         q2 = np.delete(q2, i + 1)
    #         q3 = np.delete(q3, i + 1)
    #     if q2[i] == q2[i + 1]:
    #         q1 = np.delete(q1, i + 1)
    #         q2 = np.delete(q2, i + 1)
    #         q3 = np.delete(q3, i + 1)
    #     if q3[i] == q3[i + 1]:
    #         q1 = np.delete(q1, i + 1)
    #         q2 = np.delete(q2, i + 1)
    #         q3 = np.delete(q3, i + 1)

    T = planTime(times, CartesianPoints, MoveList, Jmax, q1)
    print(T[-1])






# создаем идеальные сплайны по 3 осям
    BSplines = PrepareBSpline(q1, q2, q3, T, 1, 1.1)
    axis1tck = BSplines[0]
    testJ2 = spalde(T, BSplines[0])
    knots = BSplines[0][0]
    knots1 = BSplines[0][0]
    Coefficients = []
    Coefficients.append(PPoly.from_spline(BSplines[0]).c)
    BSplines = PrepareBSpline(q1, q2, q3, T, 2, 1.1)
    axis2tck = BSplines[1]
    knots2 = BSplines[1][0]
    Coefficients.append(PPoly.from_spline(BSplines[1]).c)
    BSplines = PrepareBSpline(q1, q2, q3, T, 3, 1.1)
    axis3tck = BSplines[2]
    knots3 = BSplines[2][0]
    Coefficients.append(PPoly.from_spline(BSplines[2]).c)
    print(len(knots1), len(knots2), len(knots3))





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
    # скорость изменения изменения ускорения
    Oq1 = []
    Oq2 = []
    Oq3 = []
    # Контурные параметры
    Vx = []
    Vy = []
    Vz = []
    Ax = []
    Ay = []
    Az = []
    testx = []
    knots = T






    print('Creating polynome splines of speed etc...')
    i = 0
    BSplines = PrepareBSpline(q1, q2, q3, T, 1, 5.1)
    t1 = spalde(T, BSplines[0])
    for i in range(len(q1) - 2):
        Vq1.append(t1[i][1])
        Aq1.append(t1[i][2])
        Jq1.append(t1[i][3])
    if np.isnan(Vq1[0]):
        print(T)
        print(Vq1)
        exit(0)
    BSplines = PrepareBSpline(q1, q2, q3, T, 2, 5.1)
    t2 = spalde(T, BSplines[1])
    for i in range(len(q2) - 2):
        Vq2.append(t2[i][1])
        Aq2.append(t2[i][2])
        Jq2.append(t2[i][3])
    BSplines = PrepareBSpline(q1, q2, q3, T, 3, 5.1)
    t3 = spalde(T, BSplines[2])
    for i in range(len(q3) - 2):
        Vq3.append(t3[i][1])
        Aq3.append(t3[i][2])
        Jq3.append(t3[i][3])







    # while (u < (len(knots)) - 4):
    #    Vq1.append((5 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 4))+ (4 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 3)) \
    #               + (3 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 2)) \
    #          + (2 * Coefficients[0][3][u] * ((knots[u] / len(knots)))) + (Coefficients[0][4][u]))
    #    Vq2.append((5 * Coefficients[1][0][u] * (knots[u] / len(knots)) ** 4) + (4 * Coefficients[1][1][u] * (knots[u] / len(knots)) ** 3) + (
    #                3 * Coefficients[1][2][u] * (knots[u] / len(knots)) ** 2) \
    #               + (2 * Coefficients[1][3][u] * (knots[u] / len(knots))) + (Coefficients[1][4][u]))
    #    Vq3.append((5 * Coefficients[2][0][u] * (knots[u] / len(knots)) ** 4) + (4 * Coefficients[2][1][u] * (knots[u] / len(knots)) ** 3) + (
    #            3 * Coefficients[2][2][u] * (knots[u] / len(knots)) ** 2) \
    #               + (2 * Coefficients[2][3][u] * (knots[u] / len(knots))) + (Coefficients[2][4][u]))
    #    Aq1.append((20 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #                12 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                           6 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #               + (2 * Coefficients[0][3][u]))
    #    Aq2.append((20 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #            12 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                       6 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #               + (2 * Coefficients[1][3][u]))
    #    Aq3.append((20 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #            12 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                       6 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #               + (2 * Coefficients[2][3][u]))
    #    Jq1.append((60 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #            24 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                       6 * Coefficients[0][2][u]))
    #    Jq2.append((60 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #            24 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                       6 * Coefficients[1][2][u]))
    #    Jq3.append((60 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #            24 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                       6 * Coefficients[2][2][u]))
    #    u += 1
    i = 3







    #print(Vq1)
    testV = Vq1
    testA = Aq1
    testJ = Jq1
    realspeed = []
    vp = []
    realspeed.append([])
    realspeed.append([])
    realspeed.append([])
    realspeed[0].append(0)
    realspeed[1].append(0)
    realspeed[2].append(0)
    temp = ForwardSpeedKins(q1, q2, Vq1, Vq2, Vq3, 175, 275, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF)
    realspeed[0] = temp[0]
    realspeed[1] = temp[1]
    realspeed[2] = temp[2]
    import plotly.graph_objects as go
    import plotly.express as px

    fig = px.scatter(x=q1, y=q2)
    fig.show()
    # for i in range(len(realspeed[2])):
    #     print(i)
    #     vp.append(math.sqrt(realspeed[0][i] ** 2 + realspeed[1][i] ** 2 + realspeed[2][i] ** 2))
    #     T = list(T)
    # while len(T) > len(vp):
    #     print(len(T), len(vp))
    #     T.pop()
    # fig = px.scatter(x=np.arange(0, len(vp), 1), y=vp)
    # fig.show()
    # fig = go.Figure(data=[go.Scatter(x=T, y=Vq1), go.Scatter(x=T, y=Vq2), go.Scatter(x=T, y=Vq3)])
    # fig.show()
    # Vq1 = []
    # Vq2 = []
    # Vq3 = []
    # Aq1 = []
    # Aq2 = []
    # Aq3 = []
    # Jq1 = []
    # Jq2 = []
    # Jq3 = []
    # BSplines = PrepareBSpline(q1, q2, q3, T, 1, 0.0)
    # temp = spalde(T, BSplines[0])
    # for i in range(len(T)):
    #     Vq1.append(temp[i][1])
    #     Aq1.append(temp[i][2])
    #     Jq1.append(temp[i][3])
    # BSplines = PrepareBSpline(q1, q2, q3, T, 2, 0.0)
    # temp = spalde(T, BSplines[1])
    # for i in range(len(T)):
    #     Vq2.append(temp[i][1])
    #     Aq2.append(temp[i][2])
    #     Jq2.append(temp[i][3])
    # BSplines = PrepareBSpline(q1, q2, q3, T, 3, 0.0)
    # temp = spalde(T, BSplines[2])
    # for i in range(len(T)):
    #     Vq3.append(temp[i][1])
    #     Aq3.append(temp[i][2])
    #     Jq3.append(temp[i][3])
    # print('LEN VQ', len(Vq1), len(Vq2), len(Vq3))
    #exit(0)








    print('Starting spline fitting...')
    s = 0.0
    T = planTime(times, CartesianPoints, MoveList, Jmax, q1)
    i = 0
    s = 1
    # здесь  начинаем подбирать коэф. сглаживания, чтобы траектория удовлетворяла ограничениям
    while i < (len(q1)):
        print(len(Jq1))
        try:
            if (abs(Jq1[i]) > (Jmax + 0.0001)):
                print('jerk1', Jq1[i], i)
                s *= 7.3
                Jq1 = []
                Aq1 = []
                Vq1 = []
                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                BSplines = PrepareBSpline(q1, q2, q3, T, 1, s)
                # Coefficients[0] = PPoly.from_spline(BSplines[0]).c
                u = 3
                knots = BSplines[0][0]
                axis1tck = BSplines[0]
                # res = RebuildSpline(Vq1, Aq1, Jq1, Coefficients, knots, 1)
                t1 = spalde(T, BSplines[0])
                for i in range(len(q1) - 2):
                    Vq1.append(t1[i][1])
                    Aq1.append(t1[i][2])
                    Jq1.append(t1[i][3])
                i = 0
                print(len(knots))
            else:
                i += 1
        except Exception as e:
            print('q1 fit error', e)
            i += 1
        try:
            if (abs(Aq1[i]) > Amax):
                print('accel1', Aq1[i], i)
                s *= 7.3
                Jq1 = []
                Aq1 = []
                Vq1 = []

                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                BSplines = PrepareBSpline(q1, q2, q3, T, 1, s)
                # Coefficients[0] = PPoly.from_spline(BSplines[0]).c
                knots = BSplines[0][0]
                axis1tck = BSplines[0]
                i = 3
                t1 = spalde(T, BSplines[0])
                for i in range(len(q1) - 2):
                    Vq1.append(t1[i][1])
                    Aq1.append(t1[i][2])
                    Jq1.append(t1[i][3])
            else:
                i += 1
        except Exception as e:
            print('q1 fit error', e)
            i += 1
    T = planTime(times, CartesianPoints, MoveList, Jmax, q2)
    i = 1
    s = 1
    while i < (len(q2)):
        #knots = utilities.generate_knot_vector(5, len(q2))
        try:
            if (abs(Jq2[i]) > (Jmax + 0.0001)):
                print('jerk2', Jq2[i], i)
                Jq2 = []
                Aq2 = []
                Vq2 = []
                s *= 7.3
                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                # print(len(T), len(q2))
                BSplines = PrepareBSpline(q1, q2, q3, T, 2, s)
                axis2tck = BSplines[1]
                # Coefficients[1] = PPoly.from_spline(BSplines[1]).c
                # u = 3
                # knots = BSplines[1][0]
                # res = RebuildSpline(Vq2, Aq2, Jq2, Coefficients, knots, 2)
                t2 = spalde(T, BSplines[1])
                for i in range(len(q2) - 2):
                    Vq2.append(t2[i][1])
                    Aq2.append(t2[i][2])
                    Jq2.append(t2[i][3])
                i = 0
            else:
                i += 1
        except Exception as e:
            print('q2 fit error', e)
            i += 1
        try:
            if (abs(Aq2[i]) > Amax):
                print('accel2', Aq1[i], i)
                Jq2 = []
                Aq2 = []
                Vq2 = []
                s *= 7.3
                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                BSplines = PrepareBSpline(q1, q2, q3, T, 2, s)
                axis2tck = BSplines[1]
                # Coefficients[1] = PPoly.from_spline(BSplines[1]).c
                # u = 3
                # knots = BSplines[1][0]
                t2 = spalde(T, BSplines[1])
                for i in range(len(q2) - 2):
                    Vq2.append(t2[i][1])
                    Aq2.append(t2[i][2])
                    Jq2.append(t2[i][3])
                i = 0
            else:
                i += 1
        except Exception as e:
            print('q2 fit error', e)
            i += 1
    T = planTime(times, CartesianPoints, MoveList, Jmax, q3)
    i = 1
    print(len(q1), len(Jq1),len(q2), len(Jq2),len(q3), len(Jq3))
    s = 1
    while i < (len(q3)):
        try:
            if (abs(Jq3[i]) > (Jmax + 0.0001)):
                print('jerk3', Jq3[i], i)
                s *= 7.3
                Jq3 = []
                Aq3 = []
                Vq3 = []
                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                BSplines = PrepareBSpline(q1, q2, q3, T, 3, s)
                axis3tck = BSplines[2]
                # Coefficients[2] = PPoly.from_spline(BSplines[0]).c
                # u = 3
                # knots = BSplines[2][0]
                # res = RebuildSpline(Vq3, Aq3, Jq3, Coefficients, knots, 3)
                t3 = spalde(T, BSplines[2])
                for i in range(len(q3) - 2):
                    Vq3.append(t3[i][1])
                    Aq3.append(t3[i][2])
                    Jq3.append(t3[i][3])
                i = 0
            else:
                i += 1
        except Exception as e:
            print('q3 fit error', e)
            i += 1
        try:
            if (abs(Aq3[i]) > Amax):
                print('accel3', Aq1[i], i)
                Jq3 = []
                Aq3 = []
                Vq3 = []
                s *= 7.3
                T = np.delete(T, i)
                q1 = np.delete(q1, i)
                q2 = np.delete(q2, i)
                q3 = np.delete(q3, i)
                BSplines = PrepareBSpline(q1, q2, q3, T, 3, s)
                axis3tck = BSplines[2]
                # Coefficients[2] = PPoly.from_spline(BSplines[0]).c
                # u = 3
                # knots = BSplines[2][0]
                # Vq3, Aq3, Jq3 = RebuildSpline(Vq3, Aq3, Jq3, Coefficients, knots)
                t3 = spalde(T, BSplines[2])
                for i in range(len(q3) - 2):
                    Vq3.append(t3[i][1])
                    Aq3.append(t3[i][2])
                    Jq3.append(t3[i][3])
                i = 0
            else:
                i += 1
        except Exception as e:
            print('q3 fit error', e)
            i += 1
       # knots = utilities.generate_knot_vector(5, len(q3))
outputpoints = []











q1der = np.array([[q1[i], Vq1[i], Aq1[i], Jq1[i]] for i in range(min(len(q1), len(Vq1), len(Aq1)))], dtype=object)
q2der = np.array([[q2[i], Vq2[i], Aq2[i], Jq2[i]] for i in range(min(len(q2), len(Vq2), len(Aq2)))], dtype=object)
q3der = np.array([[q3[i], Vq3[i], Aq3[i], Jq3[i]] for i in range(min(len(q3), len(Vq3), len(Aq3)))], dtype=object)










# if q1der[0][1] > 0:
#     q1der[0] = [q1[0], 0, 0, Jmax]
# else:
#     q1der[0] = [q1[0], 0, 0, -Jmax]
# if q1der[-1][1] > 0:
#     q1der[-1] = [q1[-1], 0, 0, -Jmax]
# else:
#     q1der[-1] = [q1[-1], 0, 0, Jmax]









realspeed =[]
realspeed.append([])
realspeed.append([])
realspeed.append([])
realspeed[0].append(0)
realspeed[1].append(0)
realspeed[2].append(0)
realpoints = []
realpoints.append([])
realpoints.append([])
realpoints.append([])
realpoints[0].append(CurrentPos[0])
realpoints[1].append(CurrentPos[1])
realpoints[2].append(CurrentPos[2])









# по пзк получаем траекторию инструмента
for i in range((min(len(q1), len(q2), len(q3)))):
    realpoints[0].append((ForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[0])
    realpoints[1].append((ForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[1])
    realpoints[2].append((ForwardKins([q1[i], q2[i], q3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[2])
print(realpoints)
# for i in range((min(len(Vq1), len(Vq2), len(Vq3)))):
#      realspeed[0].append((ForwardKins([Vq1[i], Vq2[i], Vq3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[0])
#      realspeed[1].append((ForwardKins([Vq1[i], Vq2[i], Vq3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[1])
#      realspeed[2].append((ForwardKins([Vq1[i], Vq2[i], Vq3[i]], 175, 275, 100, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF))[2])










# realspeed[0] = np.array(realpoints[0])
# realspeed[1] = np.array(realpoints[1])
# realspeed[2] = np.array(realpoints[2])
# T = np.array(planTime(times, CartesianPoints, MoveList, Jmax, q1))
# T = np.append(T, T[-1])
# #T = np.delete(T, -1)
# print(len(T), len(realspeed[0]))
# realspeed[0] = dxdt(T, realspeed[0], kind='finite_difference', k=1)
# realspeed[1] = dxdt(T, realspeed[1], kind='finite_difference', k=1)
# realspeed[2] = dxdt(T, realspeed[2], kind='finite_difference', k=1)
#print(realspeed)
x = np.arange(0, len(realspeed[0]), 1)
x1 = np.arange(0, len(realspeed[1]), 1)










# i = 1
# v = 0
# print(q1der[i][1], Vq1[i])
# q1der[i] = [q1[i], v + (0.5 * Amax * (10 / len(q1der))**2), Amax, 0]
# while (q1der[i][1] < Vq1[i]) and (i < len(Vq1) - 1):
#     print('q1', q1der[i][1], Vq1[i])
#     q1der[i] = [q1[i], v + (0.5 * Amax * (10 / len(q1der))**2), Amax, 0]
#     i += 1
#
# print((q1der[0]))
# if q2der[0][1] > 0:
#     q2der[0] = [q2[0], 0, 0, Jmax]
# else:
#     q2der[0] = [q2[0], 0, 0, -Jmax]
# if q2der[-1][1] > 0:
#     q2der[-1] = [q2[-1], 0, 0, -Jmax]
# else:
#     q2der[-1] = [q2[-1], 0, 0, Jmax]
# v = 0
# i = 1
# q2der[i] = [q2[i], v + (0.5 * Amax * (10 / len(q2der)) ** 2), Amax, 0]
# while (q2der[i][1] < Vq2[i]) and (i < len(Vq2) - 1):
#     print('q2', q2der[i][1], Vq2[i], i)
#     q2der[i] = [q2[i], v + (0.5 * Amax * (10 / len(q2der))**2), Amax, 0]
#     i += 1
#
# if q3der[0][1] > 0:
#     q3der[0] = [q3[0], 0, 0, Jmax]
# else:
#     q3der[0] = [q1[0], 0, 0, -Jmax]
# if q3der[-1][1] > 0:
#     q3der[-1] = [q1[-1], 0, 0, -Jmax]
# else:
#     q3der[-1] = [q1[-1], 0, 0, Jmax]
# v = 0
# i = 1
# q3der[i] = [q3[i], v + (0.5 * Amax * (10 / len(q3der)) ** 2), Amax, 0]
# while (q3der[i][1] < Vq3[i]) and (i < len(Vq3) - 1):
#     print('q3', q3der[i][1], Vq3[i])
#     q3der[i] = [q3[i], v + (0.5 * Amax * (10 / len(q3der))**2), Amax, 0]
#     i += 1
#
#
#
# T = T = planTime(times, CartesianPoints, MoveList, Jmax, q1der)
# print(len(T), len(q1der))
# while (len(T) != len(q1der)):
#     T = np.delete(T, -1)
# Bq1 = BPoly.from_derivatives(T, q1der)
#
# T = planTime(times, CartesianPoints, MoveList, Jmax, q2der)
# while (len(T) != len(q2der)):
#     T = np.delete(T, -1)
# Bq2 = BPoly.from_derivatives(T, q2der)
#
#
# T = planTime(times, CartesianPoints, MoveList, Jmax, q3der)
# while (len(T) != len(q3der)):
#     T = np.delete(T, -1)
# Bq3 = BPoly.from_derivatives(T, q3der)
#
# BVq1 = Bq1.derivative(1)
# BAq1 = Bq1.derivative(2)
# BJq1 = Bq1.derivative(3)
#
# BVq1 = BVq1(planTime(times, CartesianPoints, MoveList, Jmax, Vq1))
# BAq1 = BAq1(planTime(times, CartesianPoints, MoveList, Jmax, Aq1))
# BJq1 = BJq1(planTime(times, CartesianPoints, MoveList, Jmax, Jq1))
#
# BVq2 = Bq2.derivative(1)
# BAq2 = Bq2.derivative(2)
# BJq2 = Bq2.derivative(3)
#
#
# BVq2 = BVq2(planTime(times, CartesianPoints, MoveList, Jmax, Vq2))
# BAq2 = BAq2(planTime(times, CartesianPoints, MoveList, Jmax, Aq2))
# BJq2 = BJq2(planTime(times, CartesianPoints, MoveList, Jmax, Jq2))
#
# BVq3 = Bq3.derivative(1)
# BAq3 = Bq3.derivative(2)
# BJq3 = Bq3.derivative(3)
#
# BVq3 = BVq3(planTime(times, CartesianPoints, MoveList, Jmax, Vq3))
# BAq3 = BAq3(planTime(times, CartesianPoints, MoveList, Jmax, Aq3))
# BJq3 = BJq3(planTime(times, CartesianPoints, MoveList, Jmax, Jq3))
#
#
# Bq1 = Bq1(planTime(times, CartesianPoints, MoveList, Jmax, q1der))
# Bq2 = Bq2(planTime(times, CartesianPoints, MoveList, Jmax, q2der))
# Bq3 = Bq3(planTime(times, CartesianPoints, MoveList, Jmax, q2der))
#
#
# realpoints2 = []
# realpoints2.append([])
# realpoints2.append([])
# realpoints2.append([])
# realpoints2[0].append(150)
# realpoints2[1].append(200)
# realpoints2[2].append(0)
#
#
#
# realpoints = []
# realspeed = []
# realpoints.append([])
# realpoints.append([])
# realpoints.append([])
# realpoints[0].append(150)
# realpoints[1].append(200)
# realpoints[2].append(0)
# realspeed.append([])
# realspeed.append([])
# realspeed.append([])
# realspeed[0].append(0)
# realspeed[1].append(0)
# realspeed[2].append(0)
# for i in range((min(len(Bq1), len(Bq2), len(Bq3)))):
#     realpoints[0].append((ScaraForwardKins([Bq1[i], Bq2[i], Bq3[i]], 175, 275, 100))[0])
#     realpoints[1].append((ScaraForwardKins([Bq1[i], Bq2[i], Bq3[i]], 175, 275, 100))[1])
#     realpoints[2].append((ScaraForwardKins([Bq1[i], Bq2[i], Bq3[i]], 175, 275, 100))[2])
# for i in range((min(len(BVq1), len(BVq2), len(BVq3)))):
#     realspeed[0].append((ScaraForwardKins([BVq1[i], BVq2[i], BVq3[i]], 175, 275, 100))[0])
#     realspeed[1].append((ScaraForwardKins([BVq1[i], BVq2[i], BVq3[i]], 175, 275, 100))[1])
#     realspeed[2].append((ScaraForwardKins([BVq1[i], BVq2[i], BVq3[i]], 175, 275, 100))[2])
# realspeed[0] = np.array(realpoints[0])
# realspeed[1] = np.array(realpoints[1])
# realspeed[2] = np.array(realpoints[2])
# T = np.array(planTime(times, CartesianPoints, MoveList, Jmax, q1der))
# T = np.append(T, T[-1])
# print(T)
# realspeed[0] = dxdt(realspeed[0], T, kind='finite_difference', k=1)
# realspeed[1] = dxdt(realspeed[1], T, kind='finite_difference', k=1)
# realspeed[2] = dxdt(realspeed[2], T, kind='finite_difference', k=1)
# print(realspeed)
# x = np.arange(0, len(realspeed[0]), 1)
# x1 = np.arange(0, len(realspeed[1]), 1)














vp = []
j1 = []
realspeed =[]
realspeed.append([])
realspeed.append([])
realspeed.append([])
realspeed[0].append(0)
realspeed[1].append(0)
realspeed[2].append(0)
temp = ForwardSpeedKins(q1, q2, Vq1, Vq2, Vq3, 175, 275, Kinematics, re=DeltaRE, rf=DeltaRF, e=DeltaE, f=DeltaF)
realspeed[0] = temp[0]
realspeed[1] = temp[1]
realspeed[2] = temp[2]
for i in range(min(len(realspeed[0]), len(realspeed[1]), len(realspeed[2]))):
    vp.append(math.sqrt(realspeed[0][i]**2 + realspeed[1][i]**2 + realspeed[2][i]**2))
for i in range(len(testJ2)):
    j1.append(testJ2[i][4])
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
fig = px.scatter(x=realpoints[0], y=realpoints[1])
#fig = go.Figure(data=[go.Scatter(x=realpoints[0], y=realpoints[1])])
fig.show()
file = open('prg1.sgn', 'r')

file = file.read()
file = str(file)
file = file.split('\n') # здесь читаем файл с данными лазерщиков
refposxindexstart = file.index('}', file.index('Variable= Reference Position(0)'), -1) + 1
refposxindexend = file.index('===== CH5', refposxindexstart, -1)

#
refposyindexstart = file.index('}', file.index('Variable= Reference Position(1)'), -1) + 1
refposyindexend = file.index('===== CH2', refposyindexstart, -1)
#
#
# # refposxindexstart = file.index('}', file.index('Variable= Feedback Position(1)'), -1) + 1
# # refposxindexend = file.index('===== CH6', refposxindexstart, -1)
# refposxindexstart = file.index('}', file.index('Variable= Reference Position(0)'), -1) + 1
# refposxindexend = file.index('===== CH4', refposxindexstart, -1)
#
# # refposxindexstart = file.index('}', file.index('Variable= Feedback Position(0)'), -1) + 1
# # refposxindexend = file.index('===== CH2', refposxindexstart, -1)
#
# refposyindexstart = file.index('}', file.index('Variable= Reference Position(1)'), -1) + 1
# refposyindexend = file.index('===== CH8', refposyindexstart, -1)
#
#
# # refposxindexstart = file.index('}', file.index('Variable= Feedback Position(1)'), -1) + 1
# # refposxindexend = file.index('===== CH6', refposxindexstart, -1)
print(file[refposyindexstart])
# строим графики из файлов Spiiplus
refx = list(map(float, file[refposxindexstart:refposxindexend])) # преобразуем строки в инты
refy = list(map(float, file[refposyindexstart:refposyindexend]))
fig = go.Figure(data=[go.Scatter(x=refy, y=refx, name='SPiiPLus points'), go.Scatter(x=IdealpointsX, y=IdealpointsY, name='Idealr points'), go.Scatter(x=realpoints[0], y=realpoints[1], name='Interpolator points')]) #go.Scatter(x=realpoints[0], y=realpoints[1], name='Interpolator points')
fig.show()
fig = go.Figure(data=[go.Scatter(x=realpoints[0], y=realpoints[1], name='Interpolator points'), go.Scatter(x=IdealpointsX, y=IdealpointsY, name='Idealr points')]) #go.Scatter(x=realpoints[0], y=realpoints[1], name='Interpolator points')
fig.show()
refposxindexstart = file.index('}', file.index('Variable= Feedback Velocity(0)'), -1) + 1
refposxindexend = -1
refposyindexstart = file.index('}', file.index('Variable= Feedback Velocity(1)'), -1) + 1
refposyindexend = file.index('===== CH4', refposyindexstart, -1)
refspeedx = list(map(float, file[refposxindexstart:refposxindexend]))
refspeedy = list(map(float, file[refposyindexstart:refposyindexend]))
t = np.arange(0, len(refspeedx), 1)
T = planTime(times, CartesianPoints, MoveList, Jmax, Vq1)
# fig = go.Figure(data=[go.Scatter(x=t, y=refspeedx, name='SPiiPLus speed x')])
# fig.show()
# fig = go.Figure(data=[go.Scatter(x=t, y=refspeedy, name='SPiiPLus speed y')])
# fig.show()
# fig = go.Figure(data=[go.Scatter(x=T, y=Vq1, name='Axis 1 speed')])
# fig.show()
#fig = go.Figure(data=[go.Scatter(x=realpoints[0], y=realpoints[1]), go.Scatter(x=T, y=realspeed[1]), go.Scatter(x=T, y=realspeed[2])])
#T = planTime(times, CartesianPoints, MoveList, Jmax, vp[0:-1])
#T = np.delete(T, -1)
# T = np.delete(T, -1)
# T = np.delete(T, -1)
T = list(T)
while len(T) > len(vp):
    print(len(T), len(vp))
    T.pop()
fig = px.scatter(x=np.arange(0,len(vp), 1), y=vp)
fig.show()
#fig = go.Figure(data=[go.Scatter(x=T, y=BVq1), go.Scatter(x=T, y=BVq2), go.Scatter(x=T, y=BVq3)])
T = planTime(times, CartesianPoints, MoveList, Jmax, q1)
fig = px.scatter(x=T, y=q1, title='Axis 2 position')
fig.show()
T = planTime(times, CartesianPoints, MoveList, Jmax, Vq1)
fig = go.Figure(data=[go.Scatter(x=T, y=realspeed[0], name='Tool X Speed'), go.Scatter(x=T, y=realspeed[1], name='Tool Y Speed'), go.Scatter(x=T, y=realspeed[2], name='Tool Z Speed')])
fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Vq1, name='Axis 1 speed'), go.Scatter(x=T, y=Vq2, name='Axis 2 speed'), go.Scatter(x=T, y=Vq3, name='Axis 3 speed')])
fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Aq1, name='Axis 1 accel'), go.Scatter(x=T, y=Aq2, name='Axis 2 accel'), go.Scatter(x=T, y=Aq3, name='Axis 3 accel')])
fig.show()
fig = go.Figure(data=[go.Scatter(x=T, y=Jq1, name='Axis 1 jerk'), go.Scatter(x=T, y=Jq2, name='Axis 2 jerk'), go.Scatter(x=T, y=Jq3, name='Axis 3 jerk')])
fig.show()
#test2 = scipy.interpolate.splev(T, BSplines[0])

#plt.plot(T, Jq1, 'b', label='q1')
#plt.plot(T, q2, 'r', label='q2')
#plt.plot(T, q3, 'g', label='q3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
#plt.show()
T = np.arange(0, len(Vq1), 1)

#plt.plot(T, Vq1, 'b', label='Vq1')
#plt.plot(T, Vq2, 'r', label='Vq2')
#plt.plot(T, Vq3, 'g', label='Vq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
#plt.show()
#plt.plot(T, Aq1, 'b', label='Aq1')
#plt.plot(T, Aq2, 'r', label='Aq2')
#plt.plot(T, Aq3, 'g', label='Aq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
#plt.show()
#plt.plot(T, Jq1, 'b', label='Jq1')
#plt.plot(T, Jq2, 'r', label='Jq2')
#plt.plot(T, Jq3, 'g', label='Jq3')
#plt.plot(T, q1, 'r', label='label here')
plt.legend(loc='best')
#plt.show()

# теперь можно перестроить сплайны по полученным коэф и опрделеить их вдоль оси времени сервы

tempspline = BSpline(axis1tck[0], axis1tck[1], 5)
testspline = tempspline.construct_fast(axis1tck[0], axis1tck[1], axis1tck[2])
timeaxis = np.linspace(0, SumTime, int(SumTime/PosCycleTime))
Axis1FinalPos = testspline(timeaxis, 0)
Axis1FinalSpeed = testspline(timeaxis, 1)
Axis1FinalAcc = testspline(timeaxis, 2)
fig = px.scatter(x=timeaxis, y=Axis1FinalSpeed, title='Axis 1 speed')
fig.show()
file = open('axis1res.bin', 'wb')
for i in range(len(timeaxis)):
    file.write(bytearray(np.float32(Axis1FinalSpeed[i] * 10)))
file.close()
#for i in range(len(timeaxis)):
 #   file.write(str(timeaxis[i]) + ';' + str(Axis1FinalPos[i]) + ';'  + str(Axis1FinalSpeed[i]) + ';'  + str(Axis1FinalAcc[i]) + ';' + '\n')

tempspline = BSpline(axis2tck[0], axis2tck[1], 5)
testspline = tempspline.construct_fast(axis2tck[0], axis2tck[1], axis2tck[2])
timeaxis = np.linspace(0, SumTime, int(SumTime/PosCycleTime))
Axis2FinalPos = testspline(timeaxis, 0)
Axis2FinalSpeed = testspline(timeaxis, 1)
Axis2FinalAcc = testspline(timeaxis, 2)
fig = px.scatter(x=timeaxis, y=Axis2FinalSpeed, title='Axis 2 speed')
fig.show()



file = open('axis2res.bin', 'wb')
for i in range(len(timeaxis)):
    file.write(bytearray(np.float32(Axis2FinalSpeed[i] / 10)))
file.close()




tempspline = BSpline(axis3tck[0], axis3tck[1], 5)
testspline = tempspline.construct_fast(axis3tck[0], axis3tck[1], axis3tck[2])
timeaxis = np.linspace(0, SumTime, int(SumTime/PosCycleTime))
Axis3FinalPos = testspline(timeaxis, 0)
Axis3FinalSpeed = testspline(timeaxis, 1)
Axis3FinalAcc = testspline(timeaxis, 2)
fig = px.scatter(x=timeaxis, y=Axis3FinalSpeed, title='Axis 3 speed')
fig.show()



file = open('axis3res.bin', 'wb')
for i in range(len(timeaxis)):
    file.write(bytearray(np.float32(Axis3FinalSpeed[i] * 100)))
file.close()








print(np.diff(timeaxis))


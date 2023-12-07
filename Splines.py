import math
import derivative
import numpy as np
import matplotlib as plt
import plotly
import scipy.interpolate
from pygcode import Line, Machine, GCodeRapidMove
import os
from geomdl import operations
from geomdl import NURBS, fitting
from geomdl import exchange
from geomdl import utilities
from geomdl.visualization import VisMPL
from geomdl.visualization import VisPlotly
from geomdl import knotvector
import random
from scipy.interpolate import splrep, splev, spalde, splprep, InterpolatedUnivariateSpline, interp1d, LSQUnivariateSpline, UnivariateSpline
from scipy.spatial.distance import cdist, sqeuclidean

import TimeFeedratePlan


def CreateNURBSCurve(filename, pos, NumberOfPoints):
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    # Create a NURBS curve instance (full circle)
    curve = NURBS.Curve()
    # Set up curve
    curve.degree = 2
    pos[0] *= pos[3]
    pos[1] *= pos[3]
    pos[2] *= pos[3]
    curve.ctrlptsw = exchange.import_txt(filename)
    curve.ctrlptsw = [pos] + curve.ctrlptsw
    i = 0
    x = []
    y = []
    z = []
    w = []
    # while i < (len(curve.ctrlptsw) - 1):
    #     if curve.ctrlptsw[i] == curve.ctrlptsw[i+1]:
    #         curve.ctrlptsw.pop(i+1)
    #         i = 0
    #         continue
    #     i += 1
    i = 0
    for i in range(len(curve.ctrlptsw)):
        x.append(curve.ctrlptsw[i][0])
        y.append(curve.ctrlptsw[i][1])
        z.append(curve.ctrlptsw[i][2])
        w.append(curve.ctrlptsw[i][3])
    print(curve.ctrlptsw)
    import plotly.graph_objects as go
    import plotly.express as px
    #fig = px.scatter(x=x, y=y)
    #fig.show()
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlptsw))
    t = np.linspace(0, 1, NumberOfPoints)
    T = np.linspace(0, 1, len(x))
    xyz = np.array([x,y,z])
    points_a = []
    print(T.ndim)
    spline = interp1d(x=T, y=x)
    points_x = spline(t)
    spline = interp1d(x=T, y=y)
    points_y = spline(t)
    spline = interp1d(x=T, y=z)
    points_z = spline(t)
    for i in range(len(points_y)):
        points_a.append(np.array([points_x[i], points_y[i], points_z[i]]))
    points_a = np.array(points_a)
    row_indexes = np.unique(points_a, return_index=True, axis=0)[1]
    _, idx = np.unique(points_a, axis=0, return_index=True)
    points_a = points_a[np.sort(idx)]
    print(points_a)
    for a in points_a:
        a = list(a)
        #print(a)
    # print(len(res))
    # x = []
    # y = []
    # z = []
    # for i in range(len(res)):
    #     x.append(res[i][0])
    #     y.append(res[i][1])
    #     z.append(res[i][2])
    #     print(i)
    # import plotly.express as px
    # fig = px.scatter(x=x, y=y)
    # fig.show()
    #
    # # Evaluate curve
    # #operations.refine_knotvector(curve, [3])
    # curve.delta = 0.001
    # points_a = curve.evalpts
    # curve.evaluate()
    # Plot the control point polygon and the evaluated curve
    return points_a
def EvalAccuracy(pointsOld, pointsNew):
    print('EvalAcc')
    return cdist(pointsOld, pointsNew)
def OptimizeNURBS(points):
    x = []
    y = []
    z = []
    _, idx = np.unique(points, axis=0, return_index=True)
    points = points[np.sort(idx)]
    res = tuple()
    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])
        z.append(points[i][2])
    b = []
    b.append(x)
    b.append(y)
    b.append(z)
    res = splprep(b, w=None, u=None, ub=None, ue=None, k=2, task=0, s=0.1, t=None, full_output=0, nest=None, per=0, quiet=1)
    tck = res
    derivatives = spalde(tck[1], tck[0])
    res = splev(res[1], res[0])
    ideal = []
    for i in range(len(res)):
        ideal.append(list([res[0][i], res[1][i], res[2][i]]))
    i = 0
    v = []
    rang = 0
    End = False
    smoothing = 0
    j = 1
    smoothing = 1
    for i in range(len(res)):
        v.append(list([res[0][i], res[1][i], res[2][i]]))
    while not End:
        if ((derivatives[0][j][2] > 10000000) and (derivatives[1][j][2] > 10000000) and (derivatives[2][j][2] > 10000000)):
            smoothing *= 1.1
            res = splprep(b, w=None, u=None, ub=None, ue=None, k=5, task=0, s=smoothing, t=None, full_output=0, nest=None,
                          per=0, quiet=1)
            tck = res
            derivatives = spalde(tck[1], tck[0])
            res = splev(res[1], res[0])
            v = []
            j = 1
            for i in range(len(res)):
                v.append(list([res[0][i], res[1][i], res[2][i]]))
            continue
        else:
            j += 1
        if j == len(derivatives[0]) - 1:
            End = True
    print('smoothing', smoothing)
    return res

def PrepareBSpline(q1, q2, q3, T, axis, smoothing, *args, **kwargs):
    t = np.arange(0, 1, 1/len(q2))
    result = []
    s1 = []
    s2 = []
    s3 = []
    q1tck = tuple()
    q2tck = tuple()
    q3tck = tuple()
    w = kwargs.get('w')
    #w = np.ones(len(q1))
    ideal = kwargs.get('ideal')
    if (axis == 1):
        knots = np.linspace(50, 100, 10)
        print('spline', len(T), len(q1))
        print(TimeFeedratePlan.indexes)
        w = np.ones(len(q1))
        for i in range(len(TimeFeedratePlan.indexes)):
            w[i] = 0.1
        if ideal:
           s1 = InterpolatedUnivariateSpline(T, q1, k=5)
        else:
            s1 = UnivariateSpline(T, q1, s=smoothing, k=5, w=w)
        #q1tck = splrep(T, q1, w=None, xb=None, xe=None, k=5, task=0, s=0, t=None, full_output=0, per=0, quiet=1)
        #q1tck = splrep(T, q1, w=w, xb=None, xe=None, k=5, task=0, s=0, t=q1tck[0][-4:4], full_output=0, per=0, quiet=1)
    elif (axis == 2):
        w = np.ones(len(q2))
        for i in range(len(TimeFeedratePlan.indexes)):
            w[i] = 0.5
        knots = np.linspace(50, 100, 10)
        print('spline', len(T), len(q2))
        #q2tck = splrep(T, q2, w=None, xb=None, xe=None, k=5, task=0, s=0, t=None, full_output=0, per=0, quiet=1)
        #q2tck = splrep(T, q2, w=w, xb=None, xe=None, k=5, task=0, s=0, t=q2tck[0][-4:4], full_output=0, per=0, quiet=1)
        s2 = UnivariateSpline(T, q2, s=smoothing, k=5, w=w)
    else:
        w = np.ones(len(q2))
        for i in range(len(TimeFeedratePlan.indexes)):
            w[i] = 0.1
        knots = np.linspace(50, 100, 10)
        print('spline', len(T), len(q3))
        #q3tck = splrep(T, q3, w=None, xb=None, xe=None, k=5, task=0, s=0, t=None, full_output=0, per=0, quiet=1)
        #q3tck = splrep(T, q3, w=w, xb=None, xe=None, k=5, task=0, s=0, t=q3tck[0][-4:4], full_output=0, per=0, quiet=1)
        s3 = UnivariateSpline(T, q3, s=smoothing, k=5, w=w)
    result.append(s1)
    result.append(s2)
    result.append(s3)
    return result
def RebuildSpline(x, y, z, T, axis, smoothing, *args, **kwargs):
    w = kwargs.get('w')
    knots = np.linspace(T[0], T[-1], 1000)
    s1 = LSQUnivariateSpline(T, x, t=knots, w=w, k=5)
    s2 = LSQUnivariateSpline(T, y, t=knots, w=w, k=5)
    s3 = LSQUnivariateSpline(T, z, t=knots, w=w, k=5)





    # while (u < (len(knots) - 3)):
    #     if (axis == 1):
    #         Vq1.append((5 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 4)) + (
    #                 4 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 3)) \
    #                    + (3 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 2)) \
    #                    + (2 * Coefficients[0][3][u] * ((knots[u] / len(knots)))) + (Coefficients[0][4][u]))
    #         Aq1.append((20 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #                 12 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                            6 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #                    + (2 * Coefficients[0][3][u]))
    #         Jq1.append((60 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                 24 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                            6 * Coefficients[0][2][u]))
    #     if (axis == 2):
    #         Vq1.append((5 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 4)) + (
    #                 4 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 3)) \
    #                    + (3 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 2)) \
    #                    + (2 * Coefficients[1][3][u] * ((knots[u] / len(knots)))) + (Coefficients[1][4][u]))
    #         Aq1.append((20 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #                 12 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                            6 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #                    + (2 * Coefficients[1][3][u]))
    #         Jq1.append((60 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                 24 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                            6 * Coefficients[1][2][u]))
    #     if (axis == 3):
    #         Vq1.append((5 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 4)) + (
    #                 4 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 3)) \
    #                    + (3 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 2)) \
    #                    + (2 * Coefficients[2][3][u] * ((knots[u] / len(knots)))) + (Coefficients[2][4][u]))
    #         Aq1.append((20 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 3)) + (
    #                 12 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                            6 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 1)) \
    #                    + (2 * Coefficients[2][3][u]))
    #         Jq1.append((60 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 2)) + (
    #                 24 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 1)) + (
    #                            6 * Coefficients[2][2][u]))
    #     u += 1
    # return list([Vq1, Aq1, Jq1])
import math

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
from scipy.interpolate import splrep, splev, splder, splprep, UnivariateSpline, SmoothBivariateSpline, BPoly, PPoly, BSpline, spalde
from scipy.spatial.distance import cdist, sqeuclidean
def CreateNURBSCurve(filename, pos):
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
    for i in range(len(curve.ctrlptsw)):
        x.append(curve.ctrlptsw[i][0])
        y.append(curve.ctrlptsw[i][1])
        z.append(curve.ctrlptsw[i][2])
        w.append(curve.ctrlptsw[i][3])
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlptsw))
    # Evaluate curve
    #operations.refine_knotvector(curve, [3])
    curve.delta = 0.0001
    points_a = curve.evalpts
    curve.evaluate()
    # Plot the control point polygon and the evaluated curve
    return points_a
def OptimizeNURBS(points):
    x = []
    y = []
    z = []
    res = tuple()
    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])
        z.append(points[i][2])
    b = []
    b.append(x)
    b.append(y)
    b.append(z)
    res = splprep(b, w=None, u=None, ub=None, ue=None, k=5, task=0, s=0.0, t=None, full_output=0, nest=None, per=0, quiet=1)
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
    for i in range(len(res)):
        v.append(list([res[0][i], res[1][i], res[2][i]]))
    while not End:
        rang = cdist(ideal, v, 'euclidean')
        print(j, (rang[j][0]), (rang[j][1]), rang[j][2])
        if (rang[j][0] < 0.5) and (rang[j][1] < 0.5) and (rang[j][2] < 0 .5):
            smoothing += 14.1
            res = splprep(b, w=None, u=None, ub=None, ue=None, k=5, task=0, s=smoothing, t=None, full_output=0, nest=None,
                          per=0, quiet=1)
            res = splev(res[1], res[0])
            rang = []
            v = []
            j = 1
            for i in range(len(res)):
                v.append(list([res[0][i], res[1][i], res[2][i]]))
            continue
        else:
            j += 1
        if j == len(rang) - 1:
            End = True
    print('smoothing', smoothing)
    print('Error', rang[-1])
    return res

def PrepareBSpline(q1, q2, q3, T, axis, smoothing):
    t = np.arange(0, 1, 1/len(q2))
    result = []
    q1tck = tuple()
    q2tck = tuple()
    q3tck = tuple()
    if (axis == 1):
        print('spline', len(T), len(q1))
        q1tck = splrep(T, q1, w=None, xb=None, xe=None, k=5, task=0, s=smoothing, t=None, full_output=0, per=0, quiet=1)
    elif (axis == 2):
        print('spline', len(T), len(q2))
        q2tck = splrep(T, q2, w=None, xb=None, xe=None, k=5, task=0, s=smoothing, t=None, full_output=0, per=0, quiet=1)
    else:
        print('spline', len(T), len(q2))
        q3tck = splrep(T, q3, w=None, xb=None, xe=None, k=5, task=0, s=smoothing, t=None, full_output=0, per=0, quiet=1)
    result.append(q1tck)
    result.append(q2tck)
    result.append(q3tck)
    return result
def RebuildSpline(Vq1, Aq1, Jq1, Coefficients, knots, axis):
    u = 3
    while (u < (len(knots) - 3)):
        if (axis == 1):
            Vq1.append((5 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 4)) + (
                    4 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 3)) \
                       + (3 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 2)) \
                       + (2 * Coefficients[0][3][u] * ((knots[u] / len(knots)))) + (Coefficients[0][4][u]))
            Aq1.append((20 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 3)) + (
                    12 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                               6 * Coefficients[0][2][u] * ((knots[u] / len(knots)) ** 1)) \
                       + (2 * Coefficients[0][3][u]))
            Jq1.append((60 * Coefficients[0][0][u] * ((knots[u] / len(knots)) ** 2)) + (
                    24 * Coefficients[0][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                               6 * Coefficients[0][2][u]))
        if (axis == 2):
            Vq1.append((5 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 4)) + (
                    4 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 3)) \
                       + (3 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 2)) \
                       + (2 * Coefficients[1][3][u] * ((knots[u] / len(knots)))) + (Coefficients[1][4][u]))
            Aq1.append((20 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 3)) + (
                    12 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                               6 * Coefficients[1][2][u] * ((knots[u] / len(knots)) ** 1)) \
                       + (2 * Coefficients[1][3][u]))
            Jq1.append((60 * Coefficients[1][0][u] * ((knots[u] / len(knots)) ** 2)) + (
                    24 * Coefficients[1][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                               6 * Coefficients[1][2][u]))
        if (axis == 3):
            Vq1.append((5 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 4)) + (
                    4 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 3)) \
                       + (3 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 2)) \
                       + (2 * Coefficients[2][3][u] * ((knots[u] / len(knots)))) + (Coefficients[2][4][u]))
            Aq1.append((20 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 3)) + (
                    12 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 2)) + (
                               6 * Coefficients[2][2][u] * ((knots[u] / len(knots)) ** 1)) \
                       + (2 * Coefficients[2][3][u]))
            Jq1.append((60 * Coefficients[2][0][u] * ((knots[u] / len(knots)) ** 2)) + (
                    24 * Coefficients[2][1][u] * ((knots[u] / len(knots)) ** 1)) + (
                               6 * Coefficients[2][2][u]))
        u += 1
    return list([Vq1, Aq1, Jq1])
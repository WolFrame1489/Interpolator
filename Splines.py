import numpy as np
import matplotlib as plt
import plotly
from pygcode import Line, Machine, GCodeRapidMove
import os
from geomdl import operations
from geomdl import NURBS, BSpline
from geomdl import exchange
from geomdl import utilities
from geomdl.visualization import VisMPL
from geomdl.visualization import VisPlotly
from geomdl import knotvector
import random
from scipy.interpolate import splrep
def CreateNURBSCurve(filename, pos):
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    # Create a NURBS curve instance (full circle)
    curve = NURBS.Curve()
    # Set up curve
    curve.degree = 3
    pos[0] *= pos[3]
    pos[1] *= pos[3]
    pos[3] *= pos[3]
    curve.ctrlptsw = exchange.import_txt(filename)
    curve.ctrlptsw = [pos] + curve.ctrlptsw
    print(curve.ctrlptsw)

    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlptsw))
    print(curve.knotvector)
    # Evaluate curve
    #operations.refine_knotvector(curve, [3])
    curve.delta = 0.001
    points_a = curve.evalpts
    curve.evaluate()
    # Plot the control point polygon and the evaluated curve
    vis_comp = VisPlotly.VisCurve3D()
    curve.vis = vis_comp
    curve.render()
    return points_a
def PrepareBSpline(q1, q2, q3):
    t = np.arange(0, len(q1), 1)
    result = []
    q1tck = splrep(t, q1, w=None, xb=None, xe=None, k=5, task=0, s=None, t=None, full_output=0, per=0, quiet=1)
    t = np.arange(0, len(q2), 1)
    q2tck = splrep(t, q2, w=None, xb=None, xe=None, k=5, task=0, s=None, t=None, full_output=0, per=0, quiet=1)
    t = np.arange(0, len(q3), 1)
    q3tck = splrep(t, q3, w=None, xb=None, xe=None, k=5, task=0, s=None, t=None, full_output=0, per=0, quiet=1)
    result.append(q1tck)
    result.append(q2tck)
    result.append(q3tck)
    return result
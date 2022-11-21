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
    # Use a specialized knot vector
    #for i in range(curve.degree + 1):
    #    curve.knotvector.insert(0, 0)
    #i = curve.degree + 2
    #while i < (len(curve.ctrlpts) + 1):
    #    curve.knotvector.append((i + random.randint(0,2))/100)
    #    i += 1
    #for i in range(curve.degree + 1):
    #    curve.knotvector.append(1)
    #print(len(curve.ctrlpts), len(curve.knotvector))
    #print(curve.knotvector)
    #Set evaluation delta
    #print(points_a)
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
def CreateBSplineCurve(filename, pos):
    pos = pos
    return False
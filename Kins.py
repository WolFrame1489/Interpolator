import math
import numpy as np
def inside_limits(v, limits):
    return np.all([dim >= lim[0] and dim <= lim[1] for (dim, lim) in zip(v, limits)])
def InvKins(pointsIn, L1, L2, L3, limits, kins:str, *args, **kwargs):
    if kins == 'SCARA':
        pointsOut = []
        i = 1
        while (i < len(pointsIn)):
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            q3 = z
            dist_dist = x ** 2 + y ** 2
            dist = np.sqrt(dist_dist)
            if dist > L1 + L2:
                raise ValueError("Позиция недостижимма {}".format([x, y, z]))
            # print((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
            # print(x, y)
            q2 = math.acos((x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2))
            if ((x < 0) and (y < 0)):
                q2 *= -1
            q1 = math.atan2(x, y) - (math.atan((L2 * math.sin(q2)) / (L1 + L2 * math.cos(q2))))
            q2 *= -1

            if ((x >= 0) and (y >= 0)):
                q1 = math.radians(90) - q1
            if ((x < 0) and (y > 0)):
                q1 = math.radians(90) - q1
            if ((x < 0) and (y < 0)):
                q1 = math.radians(270) - q1
            if ((x > 0) and (y < 0)):
                q1 = math.radians(-90) - q1
            if ((x < 0) and (y == 0)):
                q1 = math.radians(270) + q1
            if (q1) < limits[0][0] or (q1) > limits[0][1]:
                raise ValueError("Данная точка нарушает лимиты q1 = {}".format(math.degrees(q1)))
            if (q2) < limits[1][0] or (q2) > limits[1][1]:
                raise ValueError("Данная точка нарушает лимиты q2 = {}".format(math.degrees(q2)))
            if q3 < limits[2][0] or q3 > limits[2][1]:
                raise ValueError("Данная точка нарушает лимиты q3 = {}".format((q3)))
            pointsOut.append([q1, q2, q3])

            i += 1
        return pointsOut
    elif kins == 'TRIV':
        pointsOut = []
        i = 1
        while (i < len(pointsIn)):
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            pointsOut.append([x, y, z])
            i += 1
        return pointsOut
    elif kins == 'DELTA':
        rf = kwargs.get('rf')  # длина ноги идущей к раб основанию
        re = kwargs.get('rf')  # длина ноги идущей к раб органу
        e = kwargs.get('e')  # Сторона подвижной платформы
        f = kwargs.get('f')  # Сторона неподвижной платформы
        pointsOut = []
        i = 1
        while (i < len(pointsIn)):
            cos120 = math.cos(2.0 * math.pi / 3.0)
            sin120 = math.sin(2.0 * math.pi / 3.0)
            theta1 = calcAngleYZ(pointsIn[i][0], pointsIn[i][1], pointsIn[i][2], f, e, rf, re)
            theta2 = calcAngleYZ(pointsIn[i][0] * cos120 + pointsIn[i][1] * sin120, pointsIn[i][1] * cos120 - pointsIn[i][0] * sin120, pointsIn[i][2], f, e, rf, re)  # rotate +120 deg
            theta3 = calcAngleYZ(pointsIn[i][0] * cos120 - pointsIn[i][1] * sin120, pointsIn[i][1] * cos120 + pointsIn[i][0] * sin120, pointsIn[i][2], f, e, rf, re)
            pointsOut.append([theta1, theta2, theta3])# rotate -120 deg
            i += 1
        return pointsOut
def ForwardKins(pos, a, b, c, kins:str, *args, **kwargs):   # c is in 2pi*rad/m
    if kins == 'SCARA':
        alpha = pos[0]
        beta = pos[1]
        z = pos[2]

        x = a * math.cos(alpha) + b * math.cos(alpha + beta)
        y = a * math.sin(alpha) + b * math.sin(alpha + beta)

        return (x, y, c * z)
    elif kins == 'TRIV':
        return (pos[0], pos[1], pos[2])
    elif kins == 'DELTA':
        rf = kwargs.get('rf') # длина ноги идущей к раб основанию
        re = kwargs.get('rf') # длина ноги идущей к раб органу
        e = kwargs.get('e') # Сторона подвижной платформы
        f = kwargs.get('f') # Сторона неподвижной платформы
        theta1 = pos[0]
        theta2 = pos[1]
        theta3 = pos[2]
        theta1, theta2, theta3 = math.radians(theta1), math.radians(theta2), math.radians(theta3)
        t = (f - e) * math.tan(math.degrees(30)) / 2
        # Calculate position of leg1's joint.  x1 is implicitly zero - along the axis
        y1 = -(t + rf * math.cos(theta1))
        z1 = -rf * math.sin(theta1)

        # Calculate leg2's joint position
        y2 = (t + rf * math.cos(theta2)) * math.sin(math.pi / 6)
        x2 = y2 * math.tan(math.pi / 3)
        z2 = -rf * math.sin(theta2)

        # Calculate leg3's joint position
        y3 = (t + rf * math.cos(theta3)) * math.sin(math.pi / 6)
        x3 = -y3 * math.tan(math.pi / 3)
        z3 = -rf * math.sin(theta3)

        # From the three positions in space, determine if there is a valid
        # location for the effector
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm;
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

        # discriminant
        d = b * b - 4.0 * a * c
        if d < 0:
            raise ValueError # non-existing point

        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm
        return  [x0, y0, z0]
def ScaraInvKins2(pos, a, b, c, limits, negative_elbow_angle=False):
    x = pos[0]
    y = pos[1]
    z = pos[2]

    dist_dist = x**2 + y**2
    dist = np.sqrt(dist_dist)

    # if the position is too far away, go to furthest possible point in the same direction
    if dist > a + b:
        raise ValueError("Позиция недостижима {}".format(pos))
        x /= dist + np.sqrt(a**2 + b**2)
        y /= dist + np.sqrt(a**2 + b**2)
        dist_dist = x**2 + y**2
        dist = np.sqrt(dist_dist)
    alpha = np.arctan2(y, x) - np.arccos((dist_dist + a**2 - b**2) / (2*a*dist))
    beta = np.arccos((dist_dist - a**2 - b**2) / (2 * a * b))
    print(math.degrees(alpha))
    print(math.degrees(beta))
    if not inside_limits((alpha, beta, z), limits):
        alpha = 2 * np.arctan2(y, x) - alpha
        beta *= -1
        if not inside_limits((alpha, beta, z), limits):
            raise ValueError("Данная точка нарушает лимиты {}".format(pos))
    return (alpha, beta, z/c)
def ForwardSpeedKins(q1, q2, Vq1, Vq2, Vq3, a, b, kins:str, *args, **kwargs):
    Vx = []
    Vy = []
    Vz = []
    if kins == 'SCARA':
        for i in range(len(Vq3)):
            Vz.append(-Vq3[i])
        i = 0
        for i in range(min(len(Vq1), len(Vq2))):
            temp = Vq1[i] * (-b * math.sin(q1[i] + q2[i]) - a * math.sin(q1[i]))
            temp = temp + (-b * math.sin(q1[i] + q2[i])) * Vq2[i]
            Vx.append(temp)
        i = 0
        temp = 0
        for i in range(min(len(Vq1), len(Vq2))):
            temp = Vq1[i] * (b * math.cos(q1[i] + q2[i]) - a * math.cos(q1[i]))
            temp = temp + (b * math.cos(q1[i] + q2[i])) * Vq2[i]
            Vy.append(temp)
        return (Vx, Vy, Vz)
    elif kins == 'TRIV' or 'DELTA':
        for i in range(len(Vq3)):
            Vz.append(Vq3[i])
        for i in range(len(Vq2)):
            Vy.append(Vq2[i])
        for i in range(len(Vq1)):
            Vx.append(Vq1[i])
        return (Vx, Vy, Vz)

def calcAngleYZ(x0, y0, z0, f, e, rf, re):
        y1 = -0.5 * 0.57735 * f
        y0 -= 0.5 * 0.57735 * e
        a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0)
        b = (y1-y0)/z0
        d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
        if d < 0:
            raise ValueError
        yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)
        zj = a + b*yj
        theta = 180.0 * math.atan(-zj/(y1-yj))/math.pi
        if yj>y1:
            theta += 180.0
        return theta


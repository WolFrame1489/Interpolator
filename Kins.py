import math
import numpy as np
def inside_limits(v, limits):
    return np.all([dim >= lim[0] and dim <= lim[1] for (dim, lim) in zip(v, limits)])
def calcAngle(reqcos):
    res1 = math.atan2(math.sqrt(1-math.pow(reqcos,
        2)), reqcos)
    res2 = math.atan2(math.sqrt(1-math.pow(reqcos,
        2))*(-1), reqcos)
    return (res1, res2)
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
            print(x, y, L1, L2)
            ctheta2 = ((x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2))
            stheta2 = math.sqrt(1 - math.pow(ctheta2, 2))
            ctheta1 =  (x * (L1 + L2 * ctheta2) + y * L2 * stheta2) / (x**2 + y**2)

            theta1a, theta1b = calcAngle(ctheta1)
            theta2a, theta2b = calcAngle(ctheta2)



            q1 = np.arctan2(y, x) - np.arccos((dist_dist + L2 ** 2 - L1 ** 2) / (2 * L2 * dist))
            q2 = np.arccos((dist_dist - L2 ** 2 - L1 ** 2) / (2 * L2 * L1))


            #q1 = theta1a
            #q2 = theta2a
            # q1 = q1 - q2
            # q2 = -q2




            # if ((x >= 0) and (y >= 0)):
            #     q1 = math.radians(90) - q1
            # if ((x < 0) and (y > 0)):
            #     q1 = math.radians(90) - q1
            # if ((x < 0) and (y < 0)):
            #     q1 = math.radians(270) - q1
            # if ((x > 0) and (y < 0)):
            #     q1 = math.radians(-90) - q1
            # if ((x < 0) and (y == 0)):
            #     q1 = math.radians(270) + q1
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
        i = 0
        pointsOut = []
        while (i < len(pointsIn)):
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            d2 = 180
            alpha = math.pi / 4
            d3 = 70
            d4 = 46
            d5 = 110
            l = 403
            re = 45
            theta = [0, (math.pi / 2), (7 * math.pi / 6), (11 * math.pi / 6)]
            rb = d2/math.sqrt(3) + d5 * math.cos(alpha) + d3 * math.sin(alpha)
            d = []
            k = 0
            diskr = [0,0,0,0]
            mu = [[0,0,0,0], [0,0,0,0], [0,0,0,0]]
            k = 1
            while k < 4:
                mu[1][k] = 2 * math.cos(alpha) * (re - rb + math.cos(theta[k]) * x + math.sin(theta[k]) * y) - 2 * math.sin(alpha) * z
                mu[2][k] = -(l * l) + x * x + y * y + z * z + (re - rb) * (
                            re - rb + 2 * math.cos(theta[k]) * x + 2 * math.sin(theta[k]) * y)
                diskr[k] = mu[1][k] * mu[1][k] - 4 * mu[2][k]
                k += 1
            x = 0 + (-mu[1][1] - math.sqrt(diskr[1])) / 2
            y = 0 + (-mu[1][2] - math.sqrt(diskr[2])) / 2
            z = 0 + (-mu[1][3] - math.sqrt(diskr[3])) / 2
            pointsOut.append([x, y, z])
            i += 1
    return pointsOut
def ForwardKins(pos, b, a, c, kins:str, *args, **kwargs):   # c is in 2pi*rad/m
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
        d2 = 180
        alpha = math.pi / 4
        d3 = 70
        d4 = 46
        d5 = 110
        l = 403
        re = 45
        theta = [0, (math.pi / 2), (7 * math.pi / 6), (11 * math.pi / 6)]
        rb = d2 / math.sqrt(3) + d5 * math.cos(alpha) + d3 * math.sin(alpha)
        d = []
        d.append(0 + pos[0])
        d.append(0 + pos[1])
        d.append(0 + pos[2])
        sigma = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
        lmbd = [0,0,0,0]
        i = 1
        while i < 4:
            sigma[1][i] = 2 * math.cos(theta[i]) * (re - rb + d[i - 1] * math.cos(alpha))
            sigma[2][i] = 2 * math.sin(theta[i]) * (re - rb + d[i - 1] * math.cos(alpha))
            sigma[3][i] = -2 * d[i - 1] * math.sin(alpha)
            lmbd[i] = l * l - d[i - 1] * d[i - 1] - (re - rb) * (re - rb) - 2 * d[i - 1] * re * math.cos(alpha) + 2 * d[i - 1] * rb * math.cos(alpha)
            i += 1
        a = [[0,0,0], [0,0,0], [0,0,0]]
        a[1][1] = sigma[1][1] - sigma[1][3]
        a[1][2] = sigma[2][1] - sigma[2][3]
        a[2][1] = sigma[1][2] - sigma[1][3]
        a[2][2] = sigma[2][2] - sigma[2][3]
        gamma = [0,0,0]

        deta = a[1][1] * a[2][2] - a[2][1] * a[1][2]
        gamma[1] = sigma[3][3] - sigma[3][1]
        gamma[2] = sigma[3][3] - sigma[3][2]
        la = [0,0,0]
        la[1] = lmbd[1] - lmbd[3]
        la[2] = lmbd[2] - lmbd[3]
        g1 = (a[2][2] * gamma[1] - a[1][2] * gamma[2]) / deta
        g2 = (-a[2][1] * gamma[1] + a[1][1] * gamma[2]) / deta
        e1 = (a[2][2] * la[1] - a[1][2] * la[2]) / deta
        e2 = (-a[2][1] * la[1] + a[1][1] * la[2]) / deta

        etta = g1 * g1 + g2 * g2 + 1;
        k = 2 * e1 * g1 + 2 * e2 * g2 + sigma[1][1] * g1 + sigma[2][1] * g2 + sigma[3][1];
        delta = e1 * e1 + e2 * e2 + sigma[1][1] * e1 + sigma[2][1] * e2 - lmbd[1];

        z = (-k + math.sqrt(k * k - 4 * etta * delta)) / (2 * etta)
        x = g1 * z + e1
        y = g2 * z+ e2
        return [x, y, z]

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


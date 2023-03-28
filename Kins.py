import math
import numpy as np
def inside_limits(v, limits):
    return np.all([dim >= lim[0] and dim <= lim[1] for (dim, lim) in zip(v, limits)])
def ScaraInvKins(pointsIn, L1, L2, L3, limits):
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
        q2 = math.acos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
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
        if ((x > 0) and (y <0)):
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
def ScaraForwardKins(pos, a, b, c):   # c is in 2pi*rad/m
    alpha = pos[0]
    beta = pos[1]
    z = pos[2]

    x = a * math.cos(alpha) + b * math.cos(alpha + beta)
    y = a * math.sin(alpha) + b * math.sin(alpha + beta)

    return (x, y, c*z)
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
def ScaraForwardSpeedKins(q1, q2, Vq1, Vq2, Vq3, a, b):
    Vx = []
    Vy = []
    Vz = []
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




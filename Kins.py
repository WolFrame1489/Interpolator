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
        SumLenSqrd = L1 * L1 + L2 * L2
        ProdOfLens = 2 * L1 * L2
        DiffLenSqrd = L1 * L1 - L2 * L2
        theta_S_prev = 0
        theta_E_prev = 0
        while (i < len(pointsIn)):
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            lefthand = 0
            q1 = 0
            q2 = 0
            q3 = 0
            Temp1 = x * x + y * y
            Temp2 = (Temp1 - SumLenSqrd) / ProdOfLens
            if (abs(Temp2) <= 1):
                # Inverse Kinematics
                # if (x > 0):  # always gives right hand calculation
                #     x = -x
                #     lefthand = 1
                if(x<0): #do right hand calculation(positive arc cos)
                    x=x
                    righthand=1

                theta_E = math.acos(Temp2)
                theta_Q = math.acos((Temp1 + DiffLenSqrd) / (2 * L1 * math.sqrt(Temp1)))
                arctan = math.atan2(y, x)
                theta_S = arctan - theta_Q
                # print('Q1', theta_E, 'Q2', theta_S)
                if (y < 0 and lefthand == 0):
                    theta_S = 2 * math.pi + theta_S
                if (lefthand == 1):
                    theta_E = -theta_E
                    theta_S = math.pi - theta_S
                    if (y < 0):
                        theta_S = theta_S - 2 * math.pi
                    lefthand = 0
                if (theta_S < 0 or theta_S > math.pi):
                    print("Joint0 limit exceeded! Try lowering y coordinate")
                # motor control

                q1 = theta_S
                q2 = theta_E 
                q3 = z
                theta_S_prev = q1
                theta_E_prev = q2

            if (q1) < limits[0][0] or (q1) > limits[0][1]:
                raise ValueError("Данная точка нарушает лимиты q1 = {}".format(math.degrees(q1)))
            if (q2) < limits[1][0] or (q2) > limits[1][1]:
                raise ValueError("Данная точка нарушает лимиты q2 = {}".format(math.degrees(q2)))
            if q3 < limits[2][0] or q3 > limits[2][1]:
                raise ValueError("Данная точка нарушает лимиты q3 = {}".format((q3)))
            pointsOut.append([q1, q2, q3, 0, 0, 0])
            i += 1
        return pointsOut
    elif kins == 'TRIV':
        pointsOut = []
        i = 1
        while (i < len(pointsIn)):
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            pointsOut.append([x, y, z, 0, 0, 0])
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
            pointsOut.append([x, y, z, 0 , 0, 0])
            i += 1
    elif kins == 'PUMA':
        i = 0
        pointsOut = []
        flag = False
        flag2 = False
        while (i < len(pointsIn)):
            print(i)
            a5z = 200
            a3z = 433.08
            a3y = 0
            a2z = 431.8
            a1y = -149.09
            a1z = 672
            b0 = a1y
            b1 = a3y
            a0 = a1z
            a1 = a2z
            a2 = a3z
            a6 = a5z
            Limits = []
            if not flag:
                Joints = []
                flag = False
                #temp = InvKins([[-320.0, 300.0, 0.0, 1]], 400, 250, 100, Limits, 'PUMA')
                for k in range(6):
                    Joints.append(0)
            x = pointsIn[i][0]
            y = pointsIn[i][1]
            z = pointsIn[i][2]
            ox = 0
            oy = 0
            oz = 0

            if not flag2:
                RIGHTHAND = False
                TOPFINGER = True
                TOPHAND = True
                b0 = 0
                b1 = 1
                Px = 0
                Py = -a6
                Pz = 0
                P0x = 0
                P0y = 0
                P0z = 0
                flag2 = False
            #ox = oy = oz = 0
            sina = math.sin(oy)
            cosa = math.cos(oy)
            xx = P0z * cosa - P0x * sina
            P0x = P0z * sina + P0x * cosa
            P0z = xx
            sina = math.sin(ox)
            cosa = math.cos(ox)
            xx = P0y * cosa - P0z * sina
            P0z = P0y * sina + P0z * cosa
            P0y = xx
            sina = math.sin(oz)
            cosa = math.cos(oz)
            xx = P0y * cosa - P0x * sina
            P0x = P0y * sina + P0x * cosa
            P0y = xx
            # rot( & P0.z, & P0.x, World->oy);
            # rot( & P0.y, & P0.z, World->ox);
            # rot( & P0.y, & P0.x, World->oz);
            P0x = P0x + x
            P0y = P0y + y
            P0z = P0z + z

            Alpha = atan2fz(P0x, P0y)
            sqrRxy = P0x * P0x + P0y * P0y
            Rxy = math.sqrt(sqrRxy)
            if (Rxy / math.fabs(b0 - b1) < 1.0000001):
                if (b0 < b1):
                    Alpha0 = -math.pi/ 2
                else:
                    Alpha0 = math.pi/ 2
            else:
                Alpha0=math.asin((b0 - b1) / Rxy)

            if (RIGHTHAND):
                Joints[0] = Lead(Alpha - Alpha0, Joints[0])
            else:
               Joints[0] = Lead(Alpha0 + Alpha - math.pi, Joints[0])

            sqrRxy = sqrRxy - (b0 - b1) * (b0 - b1)
            if (sqrRxy < 0):
                sqrRxy = 0
                Rxy = 0
            else:
                Rxy = math.sqrt(sqrRxy)

            Beta0 = atan2fz(P0z - a0, Rxy)
            sqrA12 = sqrRxy + (P0z - a0) * (P0z - a0)
            A = a1 * a1 + a2 * a2 - sqrA12
            B = 2 * a1 * a2
            if (math.fabs(A) >= B):
                if (not RIGHTHAND):
                    Joints[1] = math.pi / 2 - Beta0
                else:
                    Joints[1] = math.pi / 2 + Beta0
                Joints[2] = 0
                if ((math.fabs(A)) - B <= 0.000005):
                    Result = 1
                else:
                    Result = 0
            else:
                Result = 1;
                Gamma = math.acos(A / B)
                Beta = math.acos((a1 * a1 + sqrA12 -a2 * a2) / (2 * a1 * math.sqrt(sqrA12)))
                if (TOPHAND):
                    Joints[1] = math.pi/ 2 - Beta - Beta0
                    Joints[2] = math.pi - Gamma
                else:
                    Joints[1] = math.pi/ 2 - Beta0 + Beta
                    Joints[2] = Gamma - math.pi
            if (not RIGHTHAND):
                Joints[1] = -Joints[1]
                Joints[2] = -Joints[2]
            ZZx = 0
            ZZy = 1
            ZZz = 0
            YYx = 0
            YYy = 0
            YYy = 1
            YYz = 0
            sina = math.sin(oy)
            cosa = math.cos(oy)
            xx = ZZz * cosa - ZZx * sina
            ZZx = ZZz * sina + ZZx * cosa
            ZZz = xx
            xx = YYz * cosa - YYx * sina
            YYx = YYz * sina + YYx * cosa
            YYz = xx

            sina = math.sin(ox)
            cosa = math.cos(ox)
            xx = ZZy * cosa - ZZz * sina
            ZZz = ZZy * sina + ZZz * cosa
            ZZy = xx
            xx = YYy * cosa - YYz * sina
            YYz = YYz * sina + YYy * cosa
            YYy = xx

            sina = math.sin(oz)
            cosa = math.cos(oz)
            xx = ZZy * cosa - ZZx * sina
            ZZx = ZZy * sina + ZZx * cosa
            ZZy = xx
            xx = YYy * cosa - YYx * sina
            YYx = YYy * sina + YYx * cosa
            YYy = xx

            sina = math.sin(-Joints[0])
            cosa = math.cos(-Joints[0])
            xx = ZZy * cosa - ZZx * sina
            ZZx = ZZy * sina + ZZx * cosa
            ZZy = xx
            xx = YYy * cosa - YYx * sina
            YYx = YYy * sina + YYx * cosa
            YYy = xx

            sina = math.sin(-Joints[1])
            cosa = math.cos(-Joints[1])
            xx = ZZz * cosa - ZZy * sina
            ZZy = ZZz * sina + ZZy * cosa
            ZZz = xx
            xx = YYz * cosa - YYy * sina
            YYy = YYz * sina + YYy * cosa
            YYz = xx

            sina = math.sin(-Joints[2])
            cosa = math.cos(-Joints[2])
            xx = ZZz * cosa - ZZy * sina
            ZZy = ZZz * sina + ZZy * cosa
            ZZz = xx
            xx = YYz * cosa - YYy * sina
            YYy = YYz * sina + YYy * cosa
            YYz = xx
            Rxy = math.sqrt(ZZx * ZZx + ZZy * ZZy)
            if (Rxy < 0.0001):
                Joints[4]=0
            else:
                if (TOPFINGER):
                    Joints[3] = Lead(atan2fz(ZZx, ZZy), Joints[3])
                    Joints[4] = atan2fz(Rxy, ZZz)
                else:
                    Joints[3] = Lead(atan2fz(ZZx, ZZy) + math.pi, Joints[3])
                    Joints[4] = atan2fz(-Rxy, ZZz)
            sina = math.sin(-Joints[3])
            cosa = math.cos(-Joints[3])
            xx = YYy * cosa - YYx * sina
            YYx = YYy * sina + YYx * cosa
            YYy = xx
            sina = math.sin(-Joints[4])
            cosa = math.cos(-Joints[4])
            xx = YYz * cosa - YYy * sina
            YYy = YYz * sina + YYy * cosa
            YYz = xx
            Joints[5] = Lead(atan2fz(YYx, YYy), Joints[5])
            pointsOut.append(Joints)
            #print('SEXSEX', Joints)
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
    elif kins == 'PUMA':
        a5z = 200
        a3z = 433.08
        a3y = 0
        a2z = 431.8
        a1y = -149.09
        a1z = 672
        Xx = 0
        Xy = 0
        Xz = 0
        Yx = 0
        Yy = -1
        Yz = 0
        Px = 0
        Py = 0
        Pz = a5z
        Zx = 0
        Zy = 0
        Zz = 1
        ox = oy = oz = 0
        sina = math.sin(pos[5])
        cosa = math.cos(pos[5])
        xx = Yy * cosa - Yx * sina
        Yx = Yy * sina + Yx * cosa
        Yy = xx

        sina = math.sin(pos[4])
        cosa = math.cos(pos[4])
        xx = Pz * cosa - Py * sina
        Py = Pz * sina + Py * cosa
        Pz = xx
        xx = Zz * cosa - Zy * sina
        Zy = Zz * sina + Zy * cosa
        Zz = xx
        xx = Yz * cosa - Yy * sina
        Yy = Yz * sina + Yy * cosa
        Yz = xx

        sina = math.sin(pos[3])
        cosa = math.cos(pos[3])
        xx = Py * cosa - Px * sina
        Px = Py * sina + Px * cosa
        Py = xx
        xx = Zy * cosa - Zx * sina
        Zx = Zy * sina + Zx * cosa
        Zy = xx
        xx = Yy * cosa - Yx * sina
        Yx = Yy * sina + Yx * cosa
        Yy = xx

        Pz = Pz + a3z

        sina = math.sin(pos[2])
        cosa = math.cos(pos[2])
        xx = Pz * cosa - Py * sina
        Py = Pz * sina + Py * cosa
        Pz = xx
        xx = Zz * cosa - Zy * sina
        Zy = Zz * sina + Zy * cosa
        Zz = xx
        xx = Yz * cosa - Yy * sina
        Yy = Yz * sina + Yy * cosa
        Yz = xx

        Px = Px - a3y
        Pz = Pz + a2z

        sina = math.sin(pos[1])
        cosa = math.cos(pos[1])
        xx = Pz * cosa - Py * sina
        Py = Pz * sina + Py * cosa
        Pz = xx
        xx = Zz * cosa - Zy * sina
        Zy = Zz * sina + Zy * cosa
        Zz = xx
        xx = Yz * cosa - Yy * sina
        Yy = Yz * sina + Yy * cosa
        Yz = xx

        Px = Px + a1y

        sina = math.sin(pos[0])
        cosa = math.cos(pos[0])
        xx = Py * cosa - Px * sina
        Px = Py * sina + Px * cosa
        Py = xx
        xx = Zy * cosa - Zx * sina
        Zx = Zy * sina + Zx * cosa
        Zy = xx
        xx = Yy * cosa - Yx * sina
        Yx = Yy * sina + Yx * cosa
        Yy = xx

        Pz = Pz + a1z

        x = Px
        y = Py
        z = Pz

        oz = Lead(math.atan2(Zx, Zy), oz)
        sina = math.sin(oz)
        cosa = math.cos(oz)
        xx = Zx * cosa - Zy * sina
        Zy = Zy * sina + Zx * cosa
        Zx = xx
        xx = Yx * cosa - Yy * sina
        Yy = Yy * sina + Yx * cosa
        Yx = xx
        # rot2( & Z.x, & Z.y, sina, cosa);
        # rot2( & Y.x, & Y.y, sina, cosa);
        ox = Lead(atan2fz(Zz, Zy),ox)
        sina = math.sin(ox)
        cosa = math.cos(ox)
        xx = Yz * cosa - Yy * sina
        Yy = Yz * sina + Yy * cosa
        Yz = xx
        # rot( & Y.z, & Y.y, World->ox);
        oy = Lead(atan2fz(Yx, Yz) + math.pi, oy)
        #print('ZAZAZA', [x, y, z, ox, oy, oz])
        return [x, y, z, ox, oy, oz]

def ForwardSpeedKins(q1, q2, Vq1, Vq2, Vq3, a, b, kins:str, *args, **kwargs):
    Vx = []
    Vy = []
    Vz = []
    if kins == 'SCARA':
        for i in range(len(Vq3)):
            Vz.append(-Vq3[i])
        i = 0
        for i in range(min(len(Vq1), len(Vq2))):
            #temp = Vq1[i] * (-b * math.sin(q1[i] + q2[i]) - a * math.sin(q1[i]))
            #temp = temp + (-b * math.sin(q1[i] + q2[i])) * Vq2[i]
            temp = -a * Vq1[i] * math.sin(q1[i]) - b * (Vq1[i] + Vq2[i]) * math.sin(q1[i] + q2[i])
            Vx.append(temp)
        i = 0
        temp = 0
        for i in range(min(len(Vq1), len(Vq2))):
            # temp = Vq1[i] * (b * math.cos(q1[i] + q2[i]) - a * math.cos(q1[i]))
            # temp = temp + (b * math.cos(q1[i] + q2[i])) * Vq2[i]
            temp = a * Vq1[i] * math.cos(q1[i]) + b * (Vq1[i] + Vq2[i]) * math.cos(q1[i] + q2[i])
            Vy.append(temp)
        return (Vx, Vy, Vz)
    else:
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

def Lead(A, Ao):
    A = math.modf(A / 2 / math.pi)[0] * 2 * math.pi
    Ao = math.modf(A / 2 / math.pi)[0] * 2 * math.pi
    if (A - Ao > math.pi):
        A = A - 2 * math.pi
    if (A - Ao < - math.pi):
        A = A + 2 * math.pi
    return A
def atan2fz (Y,  X):
    if (math.fabs(Y) < 0.000001):
        Y = 0
    if math.fabs(X) < 0.000001:
        X=0
    return math.atan2(Y,X)
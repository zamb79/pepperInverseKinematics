import numpy as np

# Tilt of elbow rotation axis
elbowTilt = -9.0 / 180.0 * np.pi

def rotMatY(angle):
    rm = np.eye(3)
    rm[0, 0] = np.cos(angle)
    rm[0, 2] = np.sin(angle)
    rm[2, 0] = -np.sin(angle)
    rm[2, 2] = np.cos(angle)
    return rm

def rotMatX(angle):
    rm = np.eye(3)
    rm[1, 1] = np.cos(angle)
    rm[1, 2] = -np.sin(angle)
    rm[2, 1] = np.sin(angle)
    rm[2, 2] = np.cos(angle)
    return rm

def rotMatZ(angle):
    rm = np.eye(3)
    rm[0, 0] = np.cos(angle)
    rm[0, 1] = -np.sin(angle)
    rm[1, 0] = np.sin(angle)
    rm[1, 1] = np.cos(angle)
    return rm



def solveSquaredEquation(a, b, c):
    eps = 1e-8
    b2m4ac = b**2 - 4 * a * c
    if b2m4ac < eps:
        return []
    if abs(a) < eps:
        if abs(b) < eps:
            return []
        return [c/b]
    x1 = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
    x2 = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)
    return [x1, x2]


def forwardKinematics(angles):
    return 0, 0, 0

def checkJointLimits(angles):
    a = angles * 180 / np.pi
    if (a[0] < -119.5) or (a[0] > 119.5): return False
    if (a[1] < -89.5 ) or (a[1] > -0.5 ): return False
    if (a[2] < -119.5) or (a[2] > 119.5): return False
    if (a[3] <  0.5  ) or (a[3] > 89.5 ): return False
    if (a[4] < -104.5) or (a[4] > 104.5): return False
    return True

def calculateElbow(circleCenter, circleRadius, circlePlaneNormal, tcpZ, plusOrMinus = 0):
    
    cc = circleCenter
    cr = circleRadius
    
    # Solve squared equation to get elbow point [xe, ye, ze]
    # z-Coordinate of elbow is equal to z-Coordinate of TCP
    ze = tcpZ # z-coordinate of elbow is equal to z-coordinate of TCP.
    n = circlePlaneNormal # normal-vector of circle plane is n
    d = np.dot(n, cc) # parameter d of circle plane is equal to dot(n, cc)
    if (np.abs(n[0]) > np.abs(n[1])):
        B = -n[1] / n[0]
        A = d / n[0] - ze * n[2] / n[0]
        D = A - cc[0]
        s = solveSquaredEquation(B**2+1, 2*B*D - 2*cc[1], D**2 + cc[1]**2 + (ze - cc[2])**2 - cr**2)
        if len(s) == 0:
            return np.array([])
        ye = s[plusOrMinus]
        xe = B * ye + A
    else:
        B = -n[0] / n[1]
        A = d / n[1] - ze * n[2] / n[1]
        D = A - cc[1]
        s = solveSquaredEquation(B**2+1, 2*B*D - 2*cc[0], D**2 + cc[0]**2 + (ze - cc[2])**2 - cr**2)
        if len(s) == 0:
            return np.array([])
        xe = s[plusOrMinus]
        ye = B * xe + A
        
    return np.array([xe, ye, ze])

def inverseKinematics(x, y, z):

    # Define shoulder as coordinate frame origin, Tool Center 
    # Point TCP, and radii of shoulder and TCP spheres.
    s   = np.array([0, 0, 0], dtype = np.float)   # shoulder
    tcp = np.array([x, y, z], dtype = np.float)   # tool center point
    rs  = 183.304638            # radius shoulder
    rt  = 222.394835            # radius TCP

    # Calculate vector v from shoulder to TCP and the distance from shoulder to 
    # elbow plane (i.e. plane of circle).
    u     = tcp - s
    uLen  = np.linalg.norm(u)
    u0    = u / uLen
    shoulderToElbowPlaneDist = (rs**2 - rt**2 + uLen**2) / (2*uLen)
    
    # Calculate the center and radius of the circle
    cc = s + shoulderToElbowPlaneDist * u0            # circle center
    rSquared = rs**2 - shoulderToElbowPlaneDist**2
    if (rSquared < 0):
        return []
    cr = np.sqrt(rSquared) # circle radius
    
    solutions = []
    for i in range(0, 2):
        # Calculate elbow position of arm
        elb = calculateElbow(cc, cr, u0, z, i)
        print("elb: ", elb)
        if len(elb) > 0:
        
            # We know the elbow point, so now we can calculate the angles step by step
            angles = np.zeros(5)
            
            # Calculate angles 0 and 1: 
            w = np.array([elb[0], 0, elb[2]])  # vector from shoulder to elbow projected to X/Z plane
            w0 = w / np.linalg.norm(w)          # normalize vector

            v = np.copy(elb)   # vector from shoulder to elbow
            v0 = v / np.linalg.norm(v)  # normalize vector

            angles[0] = np.arctan2(-w0[2], w0[0])
            angles[1] = np.arctan2(v0[1], v0.dot(w0))

            # calculate vectors along upper and lower arms
            w = tcp - elb  # vector from elbow to TCP
            w0 = w / np.linalg.norm(w)

            # er = elbow rotation coordinate system
            er = rotMatY(angles[0]) @ rotMatZ(angles[1]) # @ rotMatY(elbowTilt)
            
            d_eh = np.dot(tcp, er[0:3,0]) - np.dot(elb, er[0:3,0])
            radius = d_eh * np.tan(-elbowTilt)
            px_dash = np.dot(tcp, er[0:3,1]) - np.dot(elb, er[0:3,1])
            py_dash = np.dot(tcp, er[0:3,2]) - np.dot(elb, er[0:3,2])
            p_dash = np.array([px_dash, py_dash])
            p_dash0 = p_dash / np.linalg.norm(p_dash)
            circleCenterOnLowerArm = elb + er[0:3,0] * d_eh
            a = radius
            c = np.linalg.norm(tcp - circleCenterOnLowerArm)
            b = np.sqrt(c**2 - a**2)
            px = b
            py = radius
            p = np.array([px, py])
            p0 = p / np.linalg.norm(p)
            angles[2] = np.arccos(np.dot(p0, p_dash0))
            if (np.cross(p0, p_dash0) < 0):
                angles[2] *= -1
            
            ### update rotation of elbow and calculate angles[3]
            er = rotMatY(angles[0]) @ rotMatZ(angles[1]) @ rotMatX(angles[2]) @ rotMatY(elbowTilt)
            angles[3] = np.arctan2(np.dot(er[0:3,1], w0), np.dot(er[0:3,0], w0))
            
            #print(round(angles[2] / np.pi * 180))


            ### calculate rotation of hand and calculate angles[4]
            hr = rotMatY(angles[0]) @ rotMatZ(angles[1]) @ rotMatX(angles[2]) @ rotMatY(elbowTilt) @ rotMatZ(angles[3])
            n = np.cross(hr[0:3,0], [0, 0, 1])
            n /= np.linalg.norm(n)
            angles[4] = -np.arctan2(np.dot(hr[0:3,1], n), np.dot(hr[0:3,2], n))
            
            ### finally: check if limits of joint angles are OK
            if (checkJointLimits(angles)):
                solutions.append(angles)
            
    return solutions


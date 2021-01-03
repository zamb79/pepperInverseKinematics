import pepperInverseKinematics as ik

x = 300
y = -100
z = -20
solutions = ik.inverseKinematics(x, y, z)
print("angles (in rad): ")
print(solutions)

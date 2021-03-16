import pepperInverseKinematics as ik
import random


for i in range(10):
    print("-------------------")
    
    x = 300 + (random.random() - 0.5) * 500
    y = 0 + (random.random() - 0.5) * 500
    z = 0 + (random.random() - 0.5) * 500
    
    solutions = ik.inverseKinematics(x, y, z)
    
    print("x =", x, "; y =", y, "; z =", z)
    print("angles (in rad): ")
    print(solutions)

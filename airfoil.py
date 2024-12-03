import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

wing_path = 'D:/Downloads/xflr5_6.57_win64/xflr5_6.57_win64/airfoils'
fin_file =  'naca0012.dat'

with open(wing_path, 'r') as file:
    data = file.read()
print(data)

# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):
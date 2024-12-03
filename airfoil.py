import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os
import pandas as pd

def wing(alpha):

    wing_folder = 'D:/Downloads/xflr5_6.57_win64/xflr5_6.57_win64/airfoils'
    wing_path = os.path.join(wing_folder, 'naca2412.csv')

    with open(wing_path, 'r') as file:
        data_wing = file.read()

    lines = data_wing.strip().split('\n')
    arraycsv = [line.split(',') for line in lines][11:]
    alphaarr = [float(value) for value in np.array(arraycsv)[:, 0]]
    clarr = [float(value) for value in np.array(arraycsv)[:, 1]]
    clalpha_interp = sp.interpolate.interp1d(alphaarr, clarr)
    # print(type(clarr)) 
    # clarr_interp = np.array([]) 
    # for element in alphaarr: 
    #     clarr_interp = np.append(clarr_interp, clalpha_interp(element)) 
    return clalpha_interp(alpha)

# print(wing(5.01))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

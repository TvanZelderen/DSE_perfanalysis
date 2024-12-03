import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os
import pandas as pd

wing_folder = 'D:/Downloads/xflr5_6.57_win64/xflr5_6.57_win64/airfoils'
airfoil_name = 'naca0010'

def f_airfoil(alpha, airfoil_name):
    file_name = airfoil_name + '-100k.csv'
    wing_path = os.path.join(wing_folder, file_name)

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


if __name__ == '__main__':
    print(f_airfoil(5.01, airfoil_name))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

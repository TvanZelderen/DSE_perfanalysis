import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os
import airfoil_database

############################################## Read Instructions #############################################
############################################## Read Instructions #############################################
############################################## Read Instructions #############################################

##### Download the airfoil csv data online from http://airfoiltools.com/airfoil/ #####
##### Rename the csv file into name_airfoil-(X)k.csv, where (X)k is the reynolds number#####

## Later you need to also input your airfoil name in deploy wing.py ##

####### Put the downloaded airfoil files in the following directory #######

wing_folder = 'airfoil_database'   ### Folder directory for Keyan ###

airfoil_name = 'naca2412'  ## airfoil name, keep it consistent with your file naming ##
reynolds = '100k'  ## reynolds number for the airfoil, keep it consistent with the data that you downloaded


def f_airfoil(airfoil_name):
    file_name = airfoil_name + '-' + reynolds + '.csv'
    wing_path = os.path.join(wing_folder, file_name)

    with open(wing_path, 'r') as file:
        data_wing = file.read()

    lines = data_wing.strip().split('\n')
    arraycsv = [line.split(',') for line in lines][11:] 
    alphaarr = np.array([float(value) for value in np.array(arraycsv)[:, 0]])  ## read the alpha column
    clarr = np.array([float(value) for value in np.array(arraycsv)[:, 1]])  ## read the cl column
    cmarr = np.array([float(value) for value in np.array(arraycsv)[:, 4]])  ## read the cm column
    cdarr = np.array([float(value) for value in np.array(arraycsv)[:, 2]])
    clalpha_interp = sp.interpolate.interp1d(alphaarr, clarr)
    cmalpha_interp = sp.interpolate.interp1d(alphaarr, cmarr)
    cd0 = float(cdarr[np.where(alphaarr == 0.0)[0][0]])
    # cm = cmarr[np.where(alphaarr == alpha)]
    print(cd0)
    return clalpha_interp, cd0, cmalpha_interp

if __name__ == '__main__':
    print(f_airfoil(airfoil_name))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

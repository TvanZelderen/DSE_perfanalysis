import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os
import pandas as pd
from pathlib import Path
# import airfoil_database

############################################## Read Instructions #############################################
############################################## Read Instructions #############################################
############################################## Read Instructions #############################################

##### Download the airfoil csv data online from http://airfoiltools.com/airfoil/ #####
##### Rename the csv file into name_airfoil-(X)k.csv, where (X)k is the reynolds number#####

## Later you need to also input your airfoil name in deploy wing.py ##

####### Put the downloaded airfoil files in the following directory #######

wing_folder = 'airfoil_database'   ### Folder directory for Keyan ###

airfoil_name = 'kc135'  ## airfoil name ##
Re1 = '30k'
Re2 = '50k'
Re3 = '100k'
Re4 = '250k'
Re5 = '500k'
Re6 = '800k'

def f_airfoil(airfoil_name):

    alphaarr = np.arange(-10, 15, 0.25)
    clinterp = np.empty((0, len(alphaarr)))
    cdinterp = np.empty((0, len(alphaarr)))
    cminterp = np.empty((0, len(alphaarr)))
    Reyarr = np.array([30000, 50000, 100000, 250000, 500000, 800000])
    cl_interpolation = np.array([])
    cd_interpolation = np.array([])
    cm_interpolation = np.array([])
    
    # print(np.shape(alphaarr))

    airfoils = [
    airfoil_name + '-' + Re1 + '.csv',
    airfoil_name + '-' + Re2 + '.csv',
    airfoil_name + '-' + Re3 + '.csv',
    airfoil_name + '-' + Re4 + '.csv',
    airfoil_name + '-' + Re5 + '.csv',
    airfoil_name + '-' + Re6 + '.csv',
    ]

    for i in range(0, len(airfoils)):  
        data_wing = pd.read_csv(os.path.join(Path('airfoil_database') , airfoils[i]))
        alpha = np.array(data_wing.iloc[:, 0])

        cl = np.array(data_wing.iloc[:, 1])
        clalpha = sp.interpolate.interp1d(alpha, cl)
        clinterp = np.vstack((clinterp, clalpha(alphaarr)))

        cd = np.array(data_wing.iloc[:, 2])
        cdalpha = sp.interpolate.interp1d(alpha, cd)
        cdinterp = np.vstack((cdinterp, cdalpha(alphaarr)))
        
        cm = np.array(data_wing.iloc[:, 4])
        cmalpha = sp.interpolate.interp1d(alpha, cm)
        cminterp = np.vstack((cminterp, cmalpha(alphaarr)))


    for i in range(0, len(alphaarr)):
        clreynolds = sp.interpolate.interp1d(Reyarr, clinterp[:, i])
        cdreynolds = sp.interpolate.interp1d(Reyarr, cdinterp[:, i])
        cmreynolds = sp.interpolate.interp1d(Reyarr, cminterp[:, i])

        cl_interpolation = np.append(cl_interpolation, clreynolds)
        cd_interpolation = np.append(cd_interpolation, cdreynolds)
        cm_interpolation = np.append(cm_interpolation, cmreynolds)

    return cl_interpolation, cd_interpolation, cm_interpolation

if __name__ == '__main__':
    airfoil_wing = 'kc135'
    clinterp, cdinterp, cminterp = f_airfoil(airfoil_name = airfoil_wing)
    print(clinterp[40](30000))

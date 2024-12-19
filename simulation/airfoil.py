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

airfoil_name = 'kc135'  ## airfoil name, keep it consistent with your file naming ##
Re1 = '30k'
Re2 = '50k'
Re3 = '100k'
Re4 = '250k'
Re5 = '500k'
Re6 = '800k'



# airfoil_files = {airfoil_30k,
#            airfoil_50k,
#            airfoil_100k, 
#            airfoil_250k, 
#            airfoil_500k, 
#            airfoil_800k}


# xflr5_data = pd.DataFrame(alpha, columns = ['alpha'])
# print(xflr5_data)

def f_airfoil(airfoil_name):
    alphaarr = np.arange(-10, 15, 0.25)
    clinterp = np.empty((0, len(alphaarr)))
    cdinterp = np.empty((0, len(alphaarr)))
    cminterp = np.empty((0, len(alphaarr)))
    Reyarr = np.array([30000, 50000, 100000, 250000, 500000, 800000])
    ultimate_solution1 = np.array([])
    ultimate_solution2 = np.array([])
    ultimate_solution3 = np.array([])


    
    print(np.shape(alphaarr))

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

        # print(clinterp)
        cd = np.array(data_wing.iloc[:, 2])
        cdalpha = sp.interpolate.interp1d(alpha, cd)
        cdinterp = np.vstack((cdinterp, cdalpha(alphaarr)))
        
        cm = np.array(data_wing.iloc[:, 4])
        cmalpha = sp.interpolate.interp1d(alpha, cm)
        cminterp = np.vstack((cminterp, cmalpha(alphaarr)))
        # print(cl)


    for i in range(0, len(alphaarr)):
        clreynolds = sp.interpolate.interp1d(Reyarr, clinterp[:, i])
        cdreynolds = sp.interpolate.interp1d(Reyarr, cdinterp[:, i])
        cmreynolds = sp.interpolate.interp1d(Reyarr, cdinterp[:, i])
        ultimate_solution1 = np.append(ultimate_solution1, clreynolds)
        ultimate_solution2 = np.append(ultimate_solution2, cdreynolds)
        ultimate_solution3 = np.append(ultimate_solution3, cmreynolds)
        # cd = data_wing['CD'].to_list()
        ultimate_solution = np.vstack((ultimate_solution1, ultimate_solution2, ultimate_solution3))

            # xflr5_data.
    # cm = cmarr[np.where(alphaarr == alpha)]
    # print(cd0)
    return ultimate_solution

if __name__ == '__main__':
    airfoil_wing = 'kc135'
    ultimate_solution = f_airfoil(airfoil_name = airfoil_wing)
    # print(np.shape(clinterp), np.shape(cminterp), np.shape(cdinterp))
    print(ultimate_solution[0, 40](30000))
    # for aoa in range(-10,11):
    #     print(clalpha_wing(aoa))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

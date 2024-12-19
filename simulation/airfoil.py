import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os
import pandas as pd
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

alpha = np.arange(-10, 15, 0.25)
xflr5_data = pd.DataFrame(alpha, columns = ['alpha'])
# print(xflr5_data)

def f_airfoil(airfoil_name):
        
    # airfoil_30k = airfoil_name + '-' + Re1 + '.csv'
    # airfoil_50k = airfoil_name + '-' + Re2 + '.csv'
    # airfoil_100k = airfoil_name + '-' + Re3 + '.csv'
    # airfoil_250k = airfoil_name + '-' + Re4 + '.csv'
    # airfoil_500k = airfoil_name + '-' + Re5 + '.csv'
    # airfoil_800k = airfoil_name + '-' + Re6 + '.csv'

    for file in os.listdir('airfoil_database'):
        
        if file.startswith(airfoil_name): 
            data_wing = pd.read_csv(file)

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
    # print(cd0)
            return clalpha_interp, cd0, cmalpha_interp

if __name__ == '__main__':
    airfoil_wing = 'kc135'
    clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(airfoil_name = airfoil_wing)

    # for aoa in range(-10,11):
    #     print(clalpha_wing(aoa))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

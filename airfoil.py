import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import os

############################################## Read Instructions #############################################
############################################## Read Instructions #############################################
############################################## Read Instructions #############################################

##### Download the airfoil csv data online from http://airfoiltools.com/airfoil/ #####
##### Rename the csv file into name_airfoil-100k.csv #####

## Later you need to also input your airfoil name in deploy wing.py ##

####### Put the downloaded airfoil files in your own directory, change the following path #######

wing_folder = 'D:/Downloads/xflr5_6.57_win64/xflr5_6.57_win64/airfoils'   ### Folder directory for Keyan ###
# wing_folder = 'C:/Documenten/AE/Y3/DSE'                                 ### Folder directory for Max ###
# wing_folder =                                                           ### Folder directory for --new user-- ###

airfoil_name = 'naca0010'  ## airfoil name, keep it consistent with your file naming ##

def f_airfoil(alpha, airfoil_name):
    file_name = airfoil_name + '-100k.csv'
    wing_path = os.path.join(wing_folder, file_name)

    with open(wing_path, 'r') as file:
        data_wing = file.read()

    lines = data_wing.strip().split('\n')
    arraycsv = [line.split(',') for line in lines][11:]  ## read file, skip the first 11 rows because they are useless information 

    cdarr = np.array([float(value) for value in np.array(arraycsv)[:, 2]])  ## read the cd column
    alphaarr = np.array([float(value) for value in np.array(arraycsv)[:, 0]])  ## read the alpha column
    clarr = np.array([float(value) for value in np.array(arraycsv)[:, 1]])  ## read the cl column
    cmarr = np.array([float(value) for value in np.array(arraycsv)[:, 4]])  ## read the cm column

    ## interpolate cl-alpha curve to make it continious ##
    clalpha_interp = sp.interpolate.interp1d(alphaarr, clarr)
    cmalpha_interp = sp.interpolate.interp1d(alphaarr, cmarr)
    # print(type(alphaarr))
    cd0 = cdarr[np.where(alphaarr == 0.0)]
    # cm = cmarr[np.where(alphaarr == alpha)]
    
    return clalpha_interp(alpha), cd0, cmalpha_interp(alpha)


if __name__ == '__main__':
    print(f_airfoil(8, airfoil_name))


# print(data_wing[])
# def wing_airfoil(wing, alpha):
#     C_L = airfoil[0] * alpha + airfoil[1]
#     return C_L

# def fin_airfoil(fin, alpha):

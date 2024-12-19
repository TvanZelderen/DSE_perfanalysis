import numpy as np
import os
from matplotlib import pyplot as plt
import csv

wing_folder = 'airfoil_database'   ### Folder directory for Keyan ###

airfoil_name = 'naca2412'  ## airfoil name, keep it consistent with your file naming ##
reynolds = '100k'  ## reynolds number for the airfoil, keep it consistent with the data that you downloaded

@cache
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
    # print(cd0)
    return clalpha_interp, cd0, cmalpha_interp

if __name__ == '__main__':
    airfoil_wing = 'ah6407'
    clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(airfoil_name = airfoil_wing)

    print(clalpha_wing)




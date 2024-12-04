import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import flight_profile
import airfoil


bank_angle = np.radians(30) # 30 degrees in radians
bank_acceleration = np.radians(30) # rad/s^2
bank_time = bank_angle / bank_acceleration
start = 0

def turn(start, bank_angle, states = flight_profile.states):
    
    

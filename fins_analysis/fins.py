from math import *
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
from utils import get_isa, get_local_speed_of_sound, mach_number

#assume no wings deployed until after stabilisation
#assume no interaction of fins & fuselage
#assume 4x flat plates (rectagle of equal surface area) 0.1m x 0.1m
#assume the women can code
#assume the fuselage is a blunt body



#initialise values for stagnation pressure calc
p_stag = 0          #stagnation pressure
p_static = 22632.1  #static pressure
gamma = 1.404       #specific heat of air at -15C (because i didnt find -50C)
M = 0.738   #Mach number
S = 0.1**2 #surface area
rho = 0.340353   #density
V = 150 #velocity

#p_stag = p_static *( (1 + ((gamma - 1)/2)*M**2) ** (gamma/(gamma -1)))      #https://en.wikipedia.org/wiki/Stagnation_pressure

#D = p_stag*S

#C_d = D/(0.5*rho*V**2*S)
#print(C_d/2)

#C_dcalc = 2.714*10**3*S/(0.5*0.03102*V**2*S) #based on NASA numbers
#print(C_dcalc/2)

D = 1.28 * 0.5 * rho * V**2 * S

print(D)

L= 0.5655 * 0.5 * 1.225 * 100**2 * S

print(L)


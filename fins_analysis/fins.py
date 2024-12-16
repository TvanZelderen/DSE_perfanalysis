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
p_static = 0        #static pressure
gamma = 1.404       #specific heat of air at -15C (because i didnt find -50C)
M = 0               #Mach number

p_stag = p_static *( (1 + ((gamma - 1)/2)*M**2) ** (gamma/(gamma -1)))      #https://en.wikipedia.org/wiki/Stagnation_pressure


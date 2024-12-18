import math as

from flight_profile import alpha *

X = L * sin(alpha) - D * cos(alpha)
Z = - L * cos(alpha) - D* sin(alpha)

C_X = C_L * sin(alpha) - C_D * cos(alpha)
C_Z = - C_L * cos(alpha) - C_D* sin(alpha)

C_X0 = - C_D    #initial, steady flight condition (the equilibrium situation)
C_Lo = - C_L


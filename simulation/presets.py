import numpy as np

############  Launch Ring Characteristics  ##########
m = 20
L_body = 0.8
L_nose = 1

############  Wing Characteristics  ###########
winglength = 1

wingchord_root = 0.13
wingchord_tip = 0.13
sweep = 0   # degrees
effective_span = np.cos(np.deg2rad(sweep)) * winglength
S_wing = (wingchord_root + wingchord_tip) * effective_span / 2
Ar_wing = effective_span ** 2 / S_wing
e = 0.65

############  Fin Characteristics  ###########
finspan = 0.1
finchord_root = 0.1
finchord_tip = 0.05

S_fin = (finchord_root + finchord_tip) * finspan / 2
Ar_fin = finspan ** 2 / S_fin

############  Physics and Atmosphere ###########
G = 9.80665
T0 = 288.15
u = 1.461E-5
C_s = 110.4

############  Starting Conditions  ###########
altitude = 27000
distance = 16000
velocity = 200










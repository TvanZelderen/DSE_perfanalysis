import numpy as np
import matplotlib.pyplot as plt
from interpol_drag import launch_vehicle_drag_coef
from airfoil import f_airfoil

alpha = 5

#### Wing characteristics ###

airfoil_wing = 'naca0714'  # Input your airfoil name (see airfoil.py for instructions)
chord = 0.1        # Chord length in meters
span = 0.7          # Wingspan in meters
S = chord * span    # Wing area in m^2
Ar_wing = span ** 2 / S  # Aspect ratio

#### Fin characteristics ###

airfoil_fin = 'naca0012'
chord_fin = 0.05
span_fin = 0.3
S_fin = chord_fin * span_fin
Ar_fin = span_fin ** 2 / S_fin
# Cm
C_M_wing = f_airfoil(alpha, airfoil_name=airfoil_wing)[2]
C_M_fin = f_airfoil(alpha, airfoil_name=airfoil_fin)[2]

S_wing = np.arange(0, 0.5, 0.001)
S_finmom = np.arange(0, 0.5, 0.001)

wingmomentarr = np.array([])
finmomentarr = np.array([])

l_cg_wing = 0.2
l_cg_fin = 0.5

for i in S_wing:
    moment_wing = C_M_wing * i *l_cg_wing
    moment_fin = C_M_fin *i * l_cg_fin

    wingmomentarr = np.append(wingmomentarr, moment_wing)
    finmomentarr = np.append(finmomentarr, moment_fin)

plt.plot(S_wing, wingmomentarr, label = 'wing')
plt.plot(S_wing, finmomentarr, label = 'fin')
plt.legend()
plt.xlabel("Area")
plt.ylabel("cm*S")
plt.show()


wingmomentarr = np.append(wingmomentarr, moment_wing)
finmomentarr = np.append(finmomentarr, moment_fin)
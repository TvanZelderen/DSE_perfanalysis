import numpy as np

dt = 0.01  # Timestep
dalpha = 0.5  # Alpha step
alpha_range = np.arange(-10, 15, dalpha)
############  Launch Ring Characteristics  ##########
mass = 20
diameter = 0.29
radius = diameter / 2
L_body = 0.8
L_nose = 1
frontal_area = radius ** 2 * np.pi

############  Wing Characteristics  ###########
wing_airfoil = 'kc135'
winglength = 1
wingchord_root = 0.13
wingchord_tip = 0.13
sweep = 45   # degrees
e = 0.65

############  Fin Characteristics  ###########
fin_airfoil = 'naca0009'
finspan = 0.1
finchord_root = 0.1
finchord_tip = 0.05

############  Physics and Atmosphere ###########
G = 9.80665
T0 = 288.15
u = 1.461E-5  # Dynamic viscousity at sea level
C_s = 110.4   # Sutherland's constant, used for kinematic viscousity calculation

############  Starting Conditions  ###########
altitude = 27000
distance = 16000
velocity = 200










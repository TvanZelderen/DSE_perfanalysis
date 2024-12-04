import numpy as np
import matplotlib.pyplot as plt
from interpol_drag import launch_vehicle_drag_coef
from airfoil import f_airfoil
import scipy as sp

########################################## Instruction ############################################
########################################## Instruction ############################################
########################################## Instruction ############################################

###################################### Read before proceed ########################################
# Before running this file, first go to airfoil.py to set up your airfoil, follow the instructions #

#### Aircraft parameters ####
e = 0.6                 # Oswald efficiency number, assumed
m = 16                  # Mass of the aircraft in kg
g = 9.80665             # Gravitational acceleration in m/s^2
W = m * g               # Weight of the aircraft in N
n_max = 2.5

#### Wing characteristics ####
airfoil_wing = 'ah6407' # Input your airfoil for the wing (see airfoil.py for instructions)
chord = 0.12            # Chord length in meters
span = 1                # Wingspan in meters
S = chord * span        # Wing area in m^2
Ar_wing = span ** 2 / S # Aspect ratio

#### Fin characteristics ####
airfoil_fin = 'naca0010'  # Input your airfoil for the fin
chord_fin = 0.05
span_fin = 0.5
S_fin = chord_fin * span_fin
Ar_fin = span_fin ** 2 / S_fin

#### Constants ####
R = 287.05

### Define realistic limits for the angle of attack ###
alpha_max_deg =  5  # Maximum realistic AoA (degrees)

#### ISA conditions ####
def rhof(Altf):
    g = 9.80665      # Gravitational acceleration (m/s^2)
    R = 287.05       # Specific gas constant for dry air (J/(kg·K))
    T0 = 288.15      # Sea-level standard temperature (K)
    P0 = 101325      # Sea-level standard pressure (Pa)

    if Altf <= 11000:
        # Troposphere 
        L = -0.0065   # Temperature lapse rate (K/m) 
        T1 = T0 + L * Altf 
        P1 = P0 * (T1 / T0) ** (-g / (L * R))
    elif Altf <= 20000:
        # Lower Stratosphere
        T1 = 216.65   # Constant temperature (K)
        L = -0.0065
        T11 = T0 + L * 11000
        P11 = P0 * (T11 / T0) ** (-g / (L * R))
        P1 = P11 * np.exp(-g * (Altf - 11000) / (R * T1))
    else:
        # Upper Stratosphere
        L = 0.001     # Temperature lapse rate (K/m)
        h_base = 20000
        T_base = 216.65
        L1 = -0.0065
        T11 = T0 + L1 * 11000
        P11 = P0 * (T11 / T0) ** (-g / (L1 * R))
        P20 = P11 * np.exp(-g * (20000 - 11000) / (R * T_base))
        T1 = T_base + L * (Altf - h_base)
        P1 = P20 * (T1 / T_base) ** (-g / (L * R))

    rho1 = P1 / (R * T1)
    return P1, rho1, T1


### Initial conditions ###

V = 200  # Initial velocity, m/s
t = 0.0
dt = 0.1
Alt = 26600.0  # Initial altitude, m
gamma = 0.0
gammedot = 0.0

### Pre-analysis variables ### 

Altarray = np.array([])
distancearr = np.array([])
alphaarray = np.array([])
gammaarray = np.array([])
tarray = np.array([])
Varray = np.array([])
Macharray = np.array([])
CLarray = np.array([])
narray = np.array([])
ldarray = np.array([])
distance = 0.0
gamma_air = 1.4


### Variable to calculate outside loop for optimisation ###
C_L_max = f_airfoil(alpha_max_deg, airfoil_name = airfoil_wing)[0]
C_d0_fin = f_airfoil(alpha = 0, airfoil_name = airfoil_fin)[1]
C_L_fin_0 = f_airfoil(alpha = 0, airfoil_name = airfoil_fin)[0]
L_required = W



while Alt > 0 and V > 0:
    P, rho, T = rhof(Alt)
    if rho <= 0:
        print("Simulation terminated due to non-physical conditions.")
        break

    q = 0.5 * rho * V ** 2  # Dynamic pressure

    # Compute required lift coefficient

    C_L_required = L_required / (q * S)

    # Limit C_L to the maximum achievable based on alpha_max 

    if C_L_required > C_L_max:
        C_L = C_L_max
        alpha = alpha_max_deg
    elif C_L_required < C_L_max:
        C_L = C_L_required
        # print(C_L_required)
    else:
        C_L = C_L_required
        alpha = alpha_max_deg

    # Compute lift and drag
    L = C_L * q * S

    a = np.sqrt(gamma_air * R * T)  # Speed of sound 
    M = V / a # Mach number

    # Drag divergence mach taken into account 
    wing_characteristics = f_airfoil(alpha, airfoil_name = airfoil_wing)

    C_D_wing_ind = wing_characteristics[1] + wing_characteristics[0] ** 2 / (np.pi * e * Ar_wing)
    C_D_fin_ind = 2 * C_d0_fin + C_L_fin_0 ** 2 / (np.pi * e * Ar_fin)
    C_D_total = C_D_wing_ind + C_D_fin_ind + launch_vehicle_drag_coef(mach = M)
    D = C_D_total * q * S

    # Compute accelerations
    Vdot = (-D - W * np.sin(gamma)) / m
    gammadot = (L - W * np.cos(gamma)) / (m * V)

    # Update velocity and flight path angle
    V += Vdot * dt
    V = max(V, 0)  # Prevent negative velocity
    gamma += gammadot * dt
    gamma = -abs(gamma)
    # Update position
    V_H = V * np.cos(gamma)
    V_v = V * np.sin(gamma)

    distance += V_H * dt
    Alt += V_v * dt  # V_v is negative, so Alt decreases
    Alt = max(Alt, 0)  # Prevent negative altitude
    # print(Alt)
    # Load factor
    n = L / W
    if abs(n) > n_max:
        n = n_max
        L = n * W
        C_L = L / (q * S)
        alpha = (C_L / C_L_alpha) + alpha_0
        if alpha > alpha_max_rad:
            alpha = alpha_max_rad
            C_L = C_L_alpha * (alpha - alpha_0)

    ld = L / D

    # Store data
    Altarray = np.append(Altarray, Alt)
    distancearr = np.append(distancearr, distance)
    alphaarray = np.append(alphaarray, alpha)
    gammaarray = np.append(gammaarray, np.degrees(gamma))
    tarray = np.append(tarray, t)
    Varray = np.append(Varray, V)
    Macharray = np.append(Macharray, M)
    CLarray = np.append(CLarray, L/(q*S))
    narray = np.append(narray, n)
    ldarray = np.append(ldarray, ld)
    t += dt

    if t > 10000:
        print("Simulation terminated due to excessive simulation time.")
        break


total_distance = distancearr[-1]  # Total distance glided



plot = False
if len(distancearr) > 0 and plot:
    
    fig, axs = plt.subplots(3, 2, figsize=(10, 8))
    axs[0,0].plot(tarray, Altarray)
    axs[0,0].set_title('time vs. altitude')
    axs[1,0].plot(tarray, gammaarray)
    axs[1,0].set_title('time vs. velocity')
    axs[2,0].plot(tarray, narray)
    axs[2,0].set_title('time vs. load factor')
    axs[0,1].plot(distancearr, Altarray)
    axs[0,1].set_title('altitude vs distance')
    axs[1,1].plot(distancearr, Varray)
    axs[1,1].set_title('velocity vs distance')
    axs[2,1].plot(tarray, CLarray)
    axs[2,1].set_title('time vs CL')
    


    # plt.figure(figsize=(10, 6))
    # plt.plot(tarray, gammaarray)
    # plt.xlabel('time')
    # plt.ylabel('gamma')
    # plt.title('gamma vs time')
    plt.grid(True)
    plt.show()

print("Lift coefficient", C_L) 
# print("lift-over-drag: ", ld)
print("Time spent: ", tarray[-1]) 

print('Landing airspeed:', Varray[-1], 'm/s')
# print('C_induced: ', C_Dinduced)
# print('C_total' , C_Dtotal)
print('Landing angle:', gammaarray[-1])

print('vertical V at lanidng:' , np.abs(Varray[-1] * np.sin(np.radians(gammaarray[-1]))))

print(f"Total distance glided: {total_distance:.2f} meters")


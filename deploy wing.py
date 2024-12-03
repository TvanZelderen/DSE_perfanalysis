import numpy as np
import matplotlib.pyplot as plt
from interpol_drag import launch_vehicle_drag_coef
from airfoil import f_airfoil
import scipy as sp

### Aircraft parameters ###
e = 0.6             # Oswald efficiency number
# C_Dbody = 0.25      # Body drag coefficient
m = 16            # Mass of the aircraft in kg
g = 9.80665         # Gravitational acceleration in m/s^2
W = m * g           # Weight of the aircraft in N
n_max = 2.5

#### Wing characteristics ###

airfoil = 'naca0010'
chord = 0.1         # Chord length in meters
span = 0.5          # Wingspan in meters
S = chord * span    # Wing area in m^2
Ar = span ** 2 / S  # Aspect ratio

#### Fin characteristics ###

### Define realistic limits for the angle of attack ###
alpha_max_deg =  5  # Maximum realistic AoA (degrees)
# alpha_min_deg = -5  # Minimum realistic AoA (degrees)

# Lift curve slope and zero-lift angle of attack
# C_L_alpha   = 6                         # Lift curve slope (per radian)
# alpha_0     = np.radians(-2.5)          # Zero-lift angle of attack in radians

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

Altarray = []
distancearr = []
alphaarray = []
gammaarray = []
tarray = []
Varray = []
Macharray = []
CLarray = []
narray = []
ldarray = []
distance = 0.0
gamma_air = 1.4


while Alt > 0 and V > 0:
    P, rho, T = rhof(Alt)
    if rho <= 0:
        print("Simulation terminated due to non-physical conditions.")
        break

    q = 0.5 * rho * V ** 2  # Dynamic pressure

    # Compute required lift coefficient
    L_required = W
    C_L_required = L_required / (q * S)

    # Limit C_L to the maximum achievable based on alpha_max 
    C_L_max = f_airfoil(alpha_max_deg, airfoil_name = airfoil)

    if C_L_required > C_L_max:
        C_L = C_L_max
        alpha = alpha_max_deg
    elif C_L_required < f_airfoil(alpha_max_deg, airfoil_name = airfoil):
        C_L = C_L_required
    else:
        C_L = C_L_required
        alpha = (C_L / C_L_alpha) + alpha_0

    # Compute lift and drag
    L = C_L * q * S


    R = 287.05
    a = np.sqrt(gamma_air * R * T)
    M = V / a # Mach number

    # Drag divergence mach taken into account 
    # Cd_total = 
    D = launch_vehicle_drag_coef(mach=M) * q * S

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
    Altarray.append(Alt)
    distancearr.append(distance)
    alphaarray.append(np.degrees(alpha))
    gammaarray.append(np.degrees(gamma))
    tarray.append(t)
    Varray.append(V)
    Macharray.append(M)
    CLarray.append(L/(q*S))
    narray.append(n)
    ldarray.append(ld)
    t += dt

    if t > 10000:
        print("Simulation terminated due to excessive simulation time.")
        break





# Total distance glided
if len(distancearr) > 0:
    total_distance = distancearr[-1]
    
    # Plot altitude vs distance
    # plt.figure(figsize=(10,6))
    # plt.plot(distancearr, Altarray)
    # plt.xlabel('Distance Traveled (m)')
    # plt.ylabel('Altitude (m)')
    # plt.title('Aircraft Glide Path')
    # plt.grid(True)
    # plt.show()

    # Plot angle of attack vs distance
    # plt.figure(figsize=(10,6))
    # plt.plot(distancearr, alphaarray)
    # plt.xlabel('Distance Traveled (m)')
    # plt.ylabel('Angle of Attack (degrees)')
    # plt.title('Angle of Attack vs Distance (Limited to 5 degrees)')
    # plt.grid(True)
    # plt.show()

    # Plot velocity vs distance
    # plt.figure(figsize=(10,6))
    # plt.plot(distancearr, Varray)
    # plt.xlabel('Distance Traveled (m)')
    # plt.ylabel('Velocity (m/s)')
    # plt.title('Velocity vs Distance')
    # plt.grid(True)
    # plt.show()

    # Plot Mach number vs distance
    # plt.figure(figsize=(10,6))
    # plt.plot(distancearr, Macharray)
    # plt.xlabel('Distance Traveled (m)')
    # plt.ylabel('Mach Number')
    # plt.title('Mach Number vs Distance')
    # plt.grid(True)
    # plt.show()

    # plt.figure(figsize=(10, 6))
    # plt.plot(distancearr, Larray)
    # plt.xlabel('Distance Traveled (m)')
    # plt.ylabel('Lift Force (N)')
    # plt.title('Lift Force vs Distance')
    # plt.grid(True)
    # plt.show()

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
    print("lift-over-drag: ", ld)
    print("Time spent: ", tarray[-1])

    print('Landing airspeed:', Varray[-1], 'm/s')

    # print('C_induced: ', C_Dinduced)
    # print('C_total' , C_Dtotal)

    print('Landing gliding angle:', gammaarray[-1])
  
    print('vertical V at lanidng:' , np.abs(Varray[-1] * np.sin(np.radians(gammaarray[-1]))))

    print(f"Total distance glided: {total_distance:.2f} meters")


else:
    print("Simulation did not produce any data.")

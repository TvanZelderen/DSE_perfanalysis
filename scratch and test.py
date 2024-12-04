import numpy as np
from airfoil import f_airfoil



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

# print(rhof(26600)[1])
 
a = np.arange(-8, 8, 0.01)

for element in a: 
    cl = f_airfoil(alpha = element, airfoil_name = 'naca0012')[0]
    if 0.98 * 0.0339 <= cl <= 1.02 * 0.0339:
        print(element)
        break

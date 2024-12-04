from math import exp, sqrt


def get_isa(altitude):
    g = 9.80665  # Gravitational acceleration (m/s^2)
    R = 287.05  # Specific gas constant for dry air (J/(kg·K))
    T0 = 288.15  # Sea-level standard temperature (K)
    P0 = 101325  # Sea-level standard pressure (Pa)

    if altitude <= 11000:
        # Troposphere
        L = -0.0065  # Temperature lapse rate (K/m)
        temperature = T0 + L * altitude
        pressure = P0 * (temperature / T0) ** (-g / (L * R))
    elif altitude <= 20000:
        # Lower Stratosphere
        temperature = 216.65  # Constant temperature (K)
        L = -0.0065
        T11 = T0 + L * 11000
        P11 = P0 * (T11 / T0) ** (-g / (L * R))
        pressure = P11 * exp(-g * (altitude - 11000) / (R * temperature))
    else:
        # Upper Stratosphere
        L = 0.001  # Temperature lapse rate (K/m)
        h_base = 20000
        T_base = 216.65
        L1 = -0.0065
        T11 = T0 + L1 * 11000
        P11 = P0 * (T11 / T0) ** (-g / (L1 * R))
        P20 = P11 * exp(-g * (20000 - 11000) / (R * T_base))
        temperature = T_base + L * (altitude - h_base)
        pressure = P20 * (temperature / T_base) ** (-g / (L * R))

    density = pressure / (R * temperature)
    return pressure, density, temperature


def get_local_speed_of_sound(altitude):
    # Speed of sound model (simplified isothermal model)
    gamma = 1.4  # heat capacity ratio
    R = 287  # specific gas constant for air in J/(kg·K)
    T = get_isa(altitude)[2]  # temperature in K
    return sqrt(gamma * R * T)  # m/s


def dynamic_pressure(velocity, altitude):
    return 0.5 * get_isa(altitude)[1] * velocity**2


def mach_number(velocity, altitude):
    return velocity / get_local_speed_of_sound(altitude)


if __name__ == "__main__":
    pass

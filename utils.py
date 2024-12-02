import math


def get_air_density(altitude):
    # Air density model (simplified exponential decay)
    rho_0 = 1.225  # sea level density in kg/m³
    scale_height = 7400  # meters
    return rho_0 * math.exp(-altitude / scale_height)  # kg/m³

def get_temperature(altitude):
    # Max alt = 32km
    lapse_rate = [-6.5, 0, 1]
    altitudes = [0, 11000, 20000, 32000]
    temperature = [288.15, 216.65, 216.65, 228.65]
    if altitude <= altitudes[1]:
        return temperature[0] + lapse_rate[0] * altitude /1000
    elif altitude <= altitudes[2]:
        return temperature[1]
    elif altitude <= altitudes[3]:
        return temperature[2] + lapse_rate[2] * (altitude - altitudes[2]) / 1000
    
def get_local_speed_of_sound(altitude):
    # Speed of sound model (simplified isothermal model)
    gamma = 1.4  # heat capacity ratio
    R = 287  # specific gas constant for air in J/(kg·K)
    T = get_temperature(altitude)  # temperature in K
    return math.sqrt(gamma * R * T)  # m/s

def get_atmosphere(altitude):
    return get_temperature(altitude), get_air_density(altitude), get_local_speed_of_sound(altitude)

def dynamic_pressure(velocity, altitude):
    return 0.5 * get_air_density(altitude) * velocity**2

def mach_number(velocity, altitude):
    return velocity / get_local_speed_of_sound(altitude)


if __name__ == "__main__":
    print(get_temperature(0))
    print(get_temperature(5000))
    print(get_temperature(10000))
    print(get_temperature(15000))
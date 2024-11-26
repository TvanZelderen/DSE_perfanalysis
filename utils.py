import math


def get_air_density(altitude):
    # Air density model (simplified exponential decay)
    # This really is very crude
    rho_0 = 1.225  # sea level density in kg/m³
    scale_height = 7400  # meters
    return rho_0 * math.exp(-altitude / scale_height)  # kg/m³


if __name__ == "__main__":
    pass

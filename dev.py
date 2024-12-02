from math import sqrt, cos, atan, pi

def local_speed_of_sound(temp:int) -> float:
    """
    Return the mach number of air at a given temperature.
    Temperature in Kelvin.
    """
    return sqrt(287 * 1.4 * temp)

def wave_drag(lander:Lander, n=4, le_sweep=pi/4, ln=0.5) -> float:
    """
    Return the wave drag on the lander.
    n: number of fins
    M: mach number
    le_sweep: leading edge sweep angle
    d_nosetip: nose tip diameter, usually 10% of the body diameter
    L: body length
    ln: nose length
    """
    d_nosetip = 0.1 * lander.reference_diameter

    M = velocity_magnitude/local_speed_of_sound(temperature)

    m_le_2 = (M * cos(le_sweep))**2
    cd0_fins = n * (0.7143/m_le_2) * ((1.2*m_le_2)**3.5 * (2.4/(2.8 * m_le_2-0.4))**2.5 -1)
    cd0_body = (1.586+1.839*M**(-2))*(atan(D/(2*ln)))**1.69 * (lander.reference_area - pi * (d_nosetip/2)**2) / lander.reference_area + (1.057 + 1.217 * M**(-2)) * pi * (d_nosetip/2)**2 / lander.reference_area
    
    return cd0_fins + cd0_body
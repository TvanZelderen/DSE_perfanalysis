from utils import dynamic_pressure, mach_number
from math import pi, cos, sin, atan

# Rocket variables
diameter = 0.29  # m
body_length = 0.7  # m
nose_cone_length = 0.5  # m

velocity = 200  # m/s
altitude = 16000  # m

# Wing variables
n = 4  # number of fins
f_surf = 0.1 * 0.1  # m^2 surface area of fins
c_r = 0.1  # m root chord
c_t = 0.1  # m tip chord
le_sweep = pi / 4  # rad, pi/4 = 45 deg
le_delta = pi / 12  # rad, pi/6 = 30 deg
fin_thickness = 0.01  # m, 0.01 = 10 mm
wing_span = 0.5  # m

# Boattail variables
bt_diameter = diameter


def fleeman_drag(
    n=n,
    f_surf=f_surf,
    altitude=altitude,
    velocity=velocity,
    le_delta=le_delta,
    le_sweep=le_sweep,
    fin_thickness=fin_thickness,
    wing_span=wing_span,
    length=body_length + nose_cone_length,
    diameter=diameter,
    nose_thickness=0.1,
    bt_diameter=bt_diameter,
):
    frontal_area = pi * diameter**2 / 4
    d_nosetip = nose_thickness * diameter
    c_mac = 2 / 3 * (c_r**2 * c_r * c_t * c_t**2) / (c_r + c_t)
    mach = mach_number(velocity, altitude)
    m_le_2 = (mach * cos(le_sweep)) ** 2
    length = body_length + nose_cone_length
    dyn_pressure = dynamic_pressure(velocity, altitude)
    bt_surface = pi * bt_diameter**2 / 4

    c_d0_fins_friction = (
        n * (f_surf / frontal_area) * 0.031123 * (mach / (dyn_pressure * c_mac)) ** 0.2
    )
    c_d0_fins_wave = (
        n
        * (0.7143 / m_le_2)
        * ((1.2 * m_le_2) * 3.5 * (2.4 / (2.8 * m_le_2 - 0.4)) * 2.5 - 1)
        * sin(le_delta) ** 2
        * cos(le_sweep)
        * fin_thickness
        * (wing_span / frontal_area)
    )

    c_d0_body_friction = (
        0.031 * (length / diameter) * (mach / (dyn_pressure * length)) ** 0.2
    )
    c_d0_body_wave = (1.586 + 1.839 * mach ** (-2)) * (
        atan(diameter / (2 * length))
    ) * 1.69 * (frontal_area - pi * (d_nosetip / 2) ** 2) / frontal_area + (
        1.057 + 1.217 * mach ** (-2)
    ) * pi * (d_nosetip / 2) * 2 / frontal_area
    c_d0_body_boattail = 0

    if mach < 1:
        c_d0_body_boattail = (0.12 + 0.13 * mach**2) * bt_surface / frontal_area * 1
    else:
        c_d0_body_boattail = 0.25 / mach * bt_surface / frontal_area * 1

    return (
        c_d0_fins_friction
        + c_d0_fins_wave
        + c_d0_body_friction
        + c_d0_body_wave
        + c_d0_body_boattail
    )


if __name__ == "__main__":
    print(fleeman_drag())

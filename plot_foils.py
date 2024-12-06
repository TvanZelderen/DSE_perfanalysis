from airfoil import f_airfoil
import matplotlib.pyplot as plt
import numpy as np

if False:
    alpha_range = np.arange(-18, 18 + 0.2, 0.2)
    clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(
        airfoil_name="ah6407" # "ah6407", "naca0012", 
    )
    cl = []
    a = []
    for alpha in alpha_range:
        try:
            cl.append(clalpha_wing(alpha))
            a.append(alpha)
        except ValueError:
            pass

    fig, ax = plt.subplots()
    ax.plot(a, cl)
    plt.show()

airfoils = {
    'ah6407': [-5, 7.3],
    'naca0012': [-10, 10],
}

def get_cla(airfoil):
    airfoil, values = airfoil
    print(airfoil, values)
    clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(
        airfoil_name=airfoil # "ah6407", "naca0012", 
    )
    min, max = values

    delta_cl = clalpha_wing(max) - clalpha_wing(min)
    delta_a = max - min
    return delta_cl / delta_a

wing_airfoil = "ah6407"
wing_span = 1.5
wing_chord = 0.14

fin_airfoil = "naca0012"
fin_span = 0.1
fin_chord = 0.1

cla = get_cla(airfoils[wing_airfoil])
clah = get_cla(airfoils[fin_airfoil])


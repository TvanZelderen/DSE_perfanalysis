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
    foil_name, values = airfoil
    clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(
        airfoil_name=foil_name # "ah6407", "naca0012", 
    )
    min, max = values

    delta_cl = clalpha_wing(max) - clalpha_wing(min)
    delta_a = max - min
    return delta_cl / delta_a

wing_airfoil = "ah6407"
wing_span = 1.5
wing_chord = 0.14
s_wing = wing_span * wing_chord
wing_sweep = np.deg2rad(65)

fin_airfoil = "naca0012"
fin_span = 0.1
fin_chord = 0.1
s_ht = fin_span * 4 * fin_chord * np.cos(np.deg2rad(45))

fuselage_effect = 0.5

wing_offset = wing_chord
fin_offset = fin_chord * 0.75

can_wing_offset = wing_chord

###########################

body_length = 1 + 0.8
l_w = 1 + wing_offset + np.sin(wing_sweep) * wing_span / 2 / 2
l_t = body_length - fin_offset - l_w 
cg_from_back = 640 # 570 - 640
l_cg = body_length - l_w - cg_from_back / 1000
ar = wing_span/wing_chord
de_da = 4 / (ar + 2)
tail_volume = s_ht * l_t / (s_wing * wing_chord)

cla = get_cla((wing_airfoil, airfoils[wing_airfoil]))
clah = get_cla((fin_airfoil, airfoils[fin_airfoil]))

wing_comp = cla * l_cg / wing_chord
tail_comp = clah * tail_volume * (1-de_da) * (1 - fuselage_effect)

cma = wing_comp - tail_comp 
neutral_point = tail_comp / cla * wing_chord

print(f"Wing comp: {wing_comp:.4f}")
print(f"Tail comp: {tail_comp:.4f}\n")

print(f"Wing location = {l_w * 1000:.2f}")
print(f"c.g. location = {l_cg * 1000:.2f} behind the wing")
print(f"n.p. location = {neutral_point * 1000:.2f} behind the wing")
print(f"n.p. location = {(neutral_point + wing_offset) * 1000:.2f} behind the nose")
print(f"Tail location = {(l_w + l_t) * 1000:.2f}")
print(f"Stability margin = {(neutral_point - l_cg) * 1000:.2f}")

print(f"Stable: {cma < 0}")

#### Canard

can_l_w = body_length - can_wing_offset
can_fin_offset = 0.5 * fin_chord + fin_span
can_l_t = can_wing_offset - 1 + can_fin_offset
can_l_cg = can_wing_offset - cg_from_back / 1000
can_tail_volume = s_ht * can_l_t / (s_wing * wing_chord)
can_neutral_point = clah / cla * can_tail_volume * wing_chord
can_cma = cla * can_l_cg / wing_chord - clah * can_tail_volume

print("\nCanard time ...")
print(f"Wing location = {can_l_w * 1000:.2f}")
print(f"c.g. location = {can_l_cg * 1000:.2f} -ve is in front of the wing")
print(f"n.p. location = {can_neutral_point * 1000:.2f}")
print(f"Tail length = {(can_l_t) * 1000:.2f} -ve is in front of the wing")
print(f"Tail location = {(1 + can_fin_offset) * 1000:.2f}")

print(f"Stable {can_cma < 0}")
# print(f"Stability margin = {(neutral_point - l_cg) * 1000:.2f}")
from pathlib import Path
import pandas as pd
from scipy.interpolate import CubicSpline
import numpy as np
from utils import get_isa, mach_number, get_reynolds_number
from airfoil import f_airfoil
from presets import wing_airfoil
from multiprocessing import Pool

clinterp, cdinterp, cminterp = f_airfoil(wing_airfoil)

def csv2df():
    script_dir = Path(__file__).parent.resolve()
    rasaero_path = script_dir / "rasaero" / "launch_ring_mach_inputs.csv"
    df = pd.read_csv(rasaero_path)
    return df


def extract_cd_cl(df):
    # Filter rows for Alpha == 0 and Alpha == 4 and select the desired columns
    data_alpha_0 = df.query("Alpha == 0")[["Mach", "Cd", "Cl"]]
    data_alpha_4 = df.query("Alpha == 4")[["Mach", "Cd", "Cl"]]

    # Extract columns directly from the filtered data
    mach = data_alpha_0["Mach"].to_numpy()
    cd_zero = data_alpha_0["Cd"].to_numpy()
    cda_body = (data_alpha_4["Cd"].to_numpy() - data_alpha_0["Cd"].to_numpy()) / 4
    cla_body = (data_alpha_4["Cl"].to_numpy()) / 4

    return mach, cd_zero, cda_body, cla_body


def interpolate(df=csv2df()):
    mach, cd_zero, cda_body, cla_body = extract_cd_cl(df)
    cd_zero_interp = CubicSpline(mach, cd_zero)
    cda_body_interp = CubicSpline(mach, cda_body)
    cla_body_interp = CubicSpline(mach, cla_body)

    return cd_zero_interp, cda_body_interp, cla_body_interp


def body_coef(mach, alpha):

    if not -10 <= alpha <= 15:
        raise ValueError("An angle of attack outside of the linear range was given.")
    cd_zero_interp, cda_body_interp, cla_body_interp = interpolate()
    body_drag_coef = cd_zero_interp(mach) + cda_body_interp(mach) * alpha
    body_lift_coef = cla_body_interp(mach) * alpha

    return body_drag_coef, body_lift_coef


def body(altitude, velocity, alpha, S=(0.29 / 2) ** 2 * np.pi):
    mach = mach_number(velocity, altitude)
    body_drag_coef, body_lift_coef = body_coef(mach, alpha)
    _, density, _ = get_isa(altitude)
    drag = 0.5 * density * velocity**2 * body_drag_coef * S
    lift = 0.5 * density * velocity**2 * body_lift_coef * S

    return lift, drag
    

def wings(altitude, velocity, alpha, clinterp = clinterp, cdinterp = cdinterp, cminterp = cminterp, chord=0.13, wingspan=1):
    if not -10 <= alpha <= 15:
        raise ValueError("An angle of attack outside of the linear range was given.")
    _, density, temperature = get_isa(altitude)
    reynolds = get_reynolds_number(density, velocity, chord, temperature)
    entry = int((alpha + 10) / 0.5)
    dyn_pressure = 0.5 * density * velocity**2
    s = chord * wingspan
    wing_factor = 0.65

    return (
        dyn_pressure * clinterp[entry](reynolds) * s * wing_factor,
        dyn_pressure * cdinterp[entry](reynolds) * s / wing_factor,
        dyn_pressure * cminterp[entry](reynolds) * s * chord,
    )  ##### I think here is the issue, if only we can think of a way to parallelise this three process

# def compute_wing_force(args):
#     func, dyn_pressure, interp, entry, reynolds, s, factor_or_chord = args
#     return func(dyn_pressure, interp, entry, reynolds, s, factor_or_chord)

# def compute_lift(dyn_pressure, clinterp, entry, reynolds, s, wing_factor):
#     return dyn_pressure * clinterp[entry](reynolds) * s * wing_factor

# def compute_drag(dyn_pressure, cdinterp, entry, reynolds, s, wing_factor):
#     return dyn_pressure * cdinterp[entry](reynolds) * s / wing_factor

# def compute_moment(dyn_pressure, cminterp, entry, reynolds, s, chord):
#     return dyn_pressure * cminterp[entry](reynolds) * s * chord

# def wings(altitude, velocity, alpha, clinterp = clinterp, cdinterp = cdinterp, cminterp = cminterp, chord=0.13, wingspan=1):
#     print(alpha)
#     if not -10 <= alpha <= 15:
#         raise ValueError("An angle of attack outside of the linear range was given.")
    
#     _, density, temperature = get_isa(altitude)
#     reynolds = get_reynolds_number(density, velocity, chord, temperature)
#     entry = int((alpha + 10) / 0.25)
#     dyn_pressure = 0.5 * density * velocity**2
#     s = chord * wingspan
#     wing_factor = 0.65

#     with Pool(processes=3) as pool:
#         results = pool.map(
#             compute_wing_force,
#             [
#                 (compute_lift, dyn_pressure, clinterp, entry, reynolds, s, wing_factor),
#                 (compute_drag, dyn_pressure, cdinterp, entry, reynolds, s, wing_factor),
#                 (compute_moment, dyn_pressure, cminterp, entry, reynolds, s, chord),
#             ],
#         )
#     print(alpha, results[0], results[1], results[2])
#     return results[0], results[1], results[2]


def brakes(altitude, velocity, S=0.01 * 4):
    _, density, _ = get_isa(altitude)
    flat_plate_cd = 1.28
    return 0.5 * density * velocity**2 * flat_plate_cd * S


def fins(altitude, velocity, delta, tail_length=0.6, S=0.01):
    # TODO: verify and change tail length
    if not -10 <= delta <= 10:
        raise ValueError(
            "A fin deflection angle outside of the linear range was given."
        )
    cnδ = 0.04
    horizontal_projection = 4 * np.sqrt(2) / 2 * S
    _, density, _ = get_isa(altitude)
    N = 0.5 * density * velocity**2 * cnδ * delta * horizontal_projection
    return N, N * tail_length


def get_forces(altitude, velocity, alpha, delta, brakes=False):
    body_lift, body_drag = body(altitude, velocity, alpha)
    wing_lift, wing_drag, wing_moment = wings(altitude, velocity, alpha)
    fin_normal, fin_moment = fins(altitude, velocity, delta)
    if brakes:
        fin_drag = brakes(altitude, velocity)
    else:
        fin_drag = 0

    # print(f"Lift: {body_lift}, {wing_lift}, {fin_normal * np.cos(delta)}")
    # print(f"Drag: {body_drag}, {wing_drag}, {fin_normal * np.sin(delta)}, {fin_drag}")

    lift = body_lift + wing_lift + fin_normal * np.cos(delta)
    drag = body_drag + wing_drag + fin_normal * np.sin(delta) + fin_drag
    moment = wing_moment + fin_moment

    return lift, drag, moment

def get_max_ld(altitude, velocity):
    max_ld = [0, 0]
    for alpha in range(-10, 11):
        lift, drag, moment = get_forces(10000, 200, alpha, 0)
        if lift/drag > max_ld[0]:
            max_ld = [lift/drag, alpha]
    return max_ld


if __name__ == "__main__":
    print(get_max_ld(5000, 100))
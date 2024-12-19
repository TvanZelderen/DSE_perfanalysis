from pathlib import Path
import pandas as pd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np
from utils import get_isa, mach_number

def csv2df():
    script_dir = Path(__file__).parent.resolve()
    rasaero_path = script_dir / "rasaero" / "launch_ring_mach_inputs.csv"
    df = pd.read_csv(rasaero_path)
    return df

def extract_cd_cl(df):
    # Filter rows for Alpha == 0 and Alpha == 4 and select the desired columns
    data_alpha_0 = df.query('Alpha == 0')[['Mach', 'Cd', 'Cl']]
    data_alpha_4 = df.query('Alpha == 4')[['Mach', 'Cd', 'Cl']]

    # Extract columns directly from the filtered data
    mach = data_alpha_0['Mach'].to_numpy()
    cd_zero = data_alpha_0['Cd'].to_numpy()
    cda_body = (data_alpha_4['Cd'].to_numpy() - data_alpha_0['Cd'].to_numpy()) / 4
    cla_body = (data_alpha_4['Cl'].to_numpy()) / 4
    
    return mach, cd_zero, cda_body, cla_body

def interpolate(df=csv2df()):
    mach, cd_zero, cda_body, cla_body = extract_cd_cl(df)
    cd_zero_interp = CubicSpline(mach, cd_zero)
    cda_body_interp = CubicSpline(mach, cda_body)
    cla_body_interp = CubicSpline(mach, cla_body)
    return cd_zero_interp, cda_body_interp, cla_body_interp

def body_coef(mach, alpha):
    if not -10 <= alpha <= 10:
        raise ValueError("An angle of attack outside of the linear range was given.")
    cd_zero_interp, cda_body_interp, cla_body_interp = interpolate()
    body_drag_coef = cd_zero_interp(mach) + cda_body_interp(mach) * alpha
    body_lift_coef = cla_body_interp(mach) * alpha
    return body_drag_coef, body_lift_coef

def body(altitude, velocity, alpha, S=(0.29/2)**2*np.pi):
    mach = mach_number(velocity, altitude)
    body_drag_coef, body_lift_coef = body_coef(mach, alpha)
    _, density, _ = get_isa(altitude)
    drag = 0.5 * density * velocity**2 * body_drag_coef * S
    lift = 0.5 * density * velocity**2 * body_lift_coef * S

    return drag, lift
    

def wings(altitude, velocity, alpha):
    if not -10 <= alpha <= 10:
        raise ValueError("An angle of attack outside of the linear range was given.")
    


def brakes(altitude, velocity, S=0.01*4):
    _, density, _ = get_isa(altitude)
    flat_plate_cd = 1.28
    return 0.5 * density * velocity**2 * flat_plate_cd * S

def fins(altitude, velocity, delta, tail_length=0.6, S=0.01):
    #TODO: verify and change tail length
    if not -10 <= delta <= 10:
        raise ValueError("A fin deflection angle outside of the linear range was given.")
    cnδ = 0.04
    horizontal_projection = 4 * np.sqrt(2)/2 * S
    _, density, _ = get_isa(altitude)
    N = 0.5 * density * velocity**2 * cnδ * delta * horizontal_projection
    return N, N*tail_length


def main(altitude, velocity, alpha, delta, brakes=False):
    pass

if __name__ == "__main__":
    print(body(2, 2))


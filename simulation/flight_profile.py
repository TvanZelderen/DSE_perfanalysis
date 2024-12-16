from interpol_drag import launch_vehicle_drag_coef
import numpy as np
from utils import get_isa, dynamic_pressure, mach_number
from math import pi
from plot import plot_flight_states
from velocity_controller import ImprovedVelocityController
from airfoil import f_airfoil

def flight_derivatives(state, params):
    """
    Calculate derivatives for longitudinal flight motion

    Parameters:
    state: [V, gamma] - current velocity and flight path angle
    params: Dictionary containing aircraft and environmental parameters
        - alpha_T: Thrust angle
    """
    V, gamma, beta = state

    # Unpack parameters
    L = params["lift"]
    D = params["drag"]
    W = params["weight"]
    g = params["gravity"]
    theta = params["theta"]

    # Calculate derivatives using the given equations
    V_dot = (-D - W * np.sin(gamma)) / (W / g)
    gamma_dot = (L * np.cos(theta) + -W * np.cos(gamma)) / (W / g * V)
    beta_dot = L * np.sin(theta) / (W / g * V)
    return np.array([V_dot, gamma_dot, beta_dot])

# Take a step using RK4
def rk4_step(state_0, derivatives_func, params, dt):
    # k1 = f(t, y)
    k1 = derivatives_func(state_0, params)

    # k2 = f(t + dt/2, y + dt/2 * k1)
    half_step_state = state_0 + 0.5 * dt * k1
    k2 = derivatives_func(half_step_state, params)

    # k3 = f(t + dt/2, y + dt/2 * k2)
    half_step_state = state_0 + 0.5 * dt * k2
    k3 = derivatives_func(half_step_state, params)

    # k4 = f(t + dt, y + dt * k3)
    full_step_state = state_0 + dt * k3
    k4 = derivatives_func(full_step_state, params)

    # y_1 = y_0 + (dt/6) * (k1 + 2k2 + 2k3 + k4)
    state_1 = state_0 + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    return state_1


dt = 0.01  # time step

# Initial state
altitude = 27000
distance = 16000
velocity = 200

mass = 20 + 13
G = 9.81

radius = 0.29 / 2
frontal_area = radius**2 * pi

wingspan = 1
wingchord = 0.13
wing_sweep_angle = np.deg2rad(0) # deg
effective_wingspan = wingspan * np.cos(wing_sweep_angle)
S_wing = wingspan * wingchord
if S_wing == 0:
    Ar_wing = 0
else:
    Ar_wing = (effective_wingspan**2 / S_wing)

def update_sweep(sweep: int):
    wing_sweep_angle = np.deg2rad(sweep)
    effective_wingspan = wingspan * np.cos(wing_sweep_angle)
    if S_wing == 0:
        Ar_wing = 0
    else:
        Ar_wing = (effective_wingspan**2 / S_wing)

e = 0.6
airfoil_wing = "kc135"
airfoil_fin = "naca0009"

clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(
    airfoil_name=airfoil_wing
)
clalpha_fin, cd0_fin, cmalpha_fin = f_airfoil(
    airfoil_name=airfoil_fin
)

# alpha_clcdmax = {}
# for alpha in range(-100, 150):
#     try:
#         cl = clalpha_wing(alpha/10)
#         cd_0 = cd0_wing
#         cd = cd_0 + cl**2/(np.pi * e * wingspan**2 / S_wing)
#         print(alpha/10, cl, cd, cd_0)
#         alpha_clcdmax[alpha/10] = cl/cd
#     except ValueError:
#         pass

# # print(alpha_clcdmax)

finspan = 0.1
fin_cr = 0.1
fin_ct = 0.1
S_fin = finspan * (fin_cr + fin_ct) / 2
if S_fin == 0:
    Ar_fin = 0
else:
    Ar_fin = finspan**2 / S_fin

states = {
    'time': [],
    'velocity': [],
    'gamma': [], # Pitch angle
    'beta': [], # Yaw angle (turn)
    'theta': [], # Bank angle
    'altitude': [],
    'mach': [],
    'alpha': [],
    'vertical_speed': [],
    'distance': [],
    'wing_sweep_angle': [],
    'ar_wing': [],
    'lift': [],
    'drag': [],
    'drag_fins': [],
    'drag_wings': [],
    'drag_wave': [],
    'load_factor': [],
    'x': [],
    'y': [],
    'air_density': [],
    'cl': [],
    'wing_moment': [],
    'dyn_press': [],
}

velocity_controller = ImprovedVelocityController(
    target_velocity=80,
    min_angle_of_attack=np.deg2rad(-7.5),
    max_angle_of_attack=np.deg2rad(14.0),
)

landed = False
turn = True
spiral = False
max_bank_angle = np.deg2rad(45)

landing_angle = np.deg2rad(15)
landing_sequence = False


def initialize_states():
    # Set initial conditions
    states['time'].append(0)
    states['weight'] = [mass * G]
    states['gravity'] = [G]
    states['velocity'].append(velocity)
    states['mach'].append(mach_number(velocity, altitude))
    states['gamma'].append(0) 
    states['beta'].append(0)
    states['theta'].append(0)
    states['altitude'].append(altitude)
    states['distance'].append(distance)
    states['wing_sweep_angle'].append(wing_sweep_angle)
    states['ar_wing'].append(Ar_wing)
    states['lift'].append(0)
    states['drag'].append(0)
    states['drag_fins'].append(0)
    states['drag_wings'].append(0)
    states['drag_wave'].append(0)
    states['alpha'].append(0)
    states['vertical_speed'].append(0)
    states['load_factor'].append(0)
    states['x'].append(distance)
    states['y'].append(0)
    states['air_density'].append(0)
    states['cl'].append(0)
    states['wing_moment'].append(0)
    states['dyn_press'].append(0)

print("Starting sim...")
initialize_states()
print("Initialised states")

while not landed: 
    current_state = {key: value[-1] for key, value in states.items()}
    state = (current_state["velocity"], current_state["gamma"], current_state["beta"])

    pressure, density, temperature = get_isa(current_state["altitude"])
    dyn_press = dynamic_pressure(current_state["velocity"], current_state["altitude"])
    mach = mach_number(current_state["velocity"], current_state["altitude"])

    alpha = velocity_controller.update(current_state["velocity"], dt)

    cla_fin = float(clalpha_fin(0))
    if S_fin == 0:
        induced_drag_fins = 0
    else:
        induced_drag_fins = (
            dyn_press * (cd0_fin + cla_fin**2 / (np.pi * e * Ar_fin)) * S_fin
        )
    cla_wing = float(clalpha_wing(alpha))
    if S_wing == 0:
        induced_drag_wing = 0
    else:
        induced_drag_wing = (
            dyn_press * (cd0_wing + cla_wing**2 / (np.pi * e * Ar_wing)) * S_wing
        )
    drag_body = dyn_press * frontal_area * launch_vehicle_drag_coef(mach)
    drag = induced_drag_fins + induced_drag_wing + drag_body
    states['drag_fins'].append(induced_drag_fins)
    states['drag_wings'].append(induced_drag_wing)
    states['drag_wave'].append(drag_body)

    cma = float(cmalpha_wing(alpha))
    wing_moment = cma * dyn_press * S_wing * wingchord

    ########### Turn Implementation ##########
    if current_state["beta"] >= np.pi and turn: # Initial turn to 0 x distance
        turn = False
        print("Backtrack turn completed")
    if current_state['x'] <= 0 and not spiral and not landing_sequence: # Once at 0 x distance, start the spiral down
        spiral = True
        print("Spiral started")
    if current_state['altitude'] <= np.tan(landing_angle) * np.sqrt(current_state['x']**2 + current_state['y']**2) and not landing_sequence:
        landing_sequence = True
        update_sweep(0)
        print("Landing sequence started")
    if landing_sequence:
        target_angle = np.arctan2(-current_state['y'], -current_state['x'])
        alpha = np.deg2rad(5)
        spiral = False

    horizontal_speed = current_state["velocity"] * np.cos(current_state["gamma"])
    if turn or spiral:
        current_state['theta'] = max_bank_angle
    elif landing_sequence:
        if (current_state['beta'] - target_angle) % np.deg2rad(360) < pi: # Turn right
            current_state['theta'] = - max_bank_angle
        else: # Turn left
            current_state['theta'] = max_bank_angle
    else:
        current_state['theta'] = 0
    
    horizontal_new = horizontal_speed * np.cos(current_state['beta'])
    horizontal_side = horizontal_speed * np.sin(current_state['beta'])
    distance = distance = (
        current_state["distance"] + horizontal_new * dt
    )  # V * cos(gamma)
    lift = dyn_press * clalpha_wing(np.rad2deg(alpha)) * S_wing * np.cos(current_state['theta'])

    load_factor = dyn_press * clalpha_wing(np.rad2deg(alpha)) * S_wing / current_state['weight']

    dx = current_state['velocity'] * np.cos(current_state['gamma']) * np.cos(current_state['beta']) * dt
    dy = current_state['velocity'] * np.cos(current_state['gamma']) * np.sin(current_state['beta']) * dt
    x = current_state['x'] + dx
    y = current_state['y'] + dy

    states["lift"].append(lift)
    states["drag"].append(drag)
    states['wing_moment'].append(wing_moment)

    current_state["lift"] = states["lift"][-1]
    current_state["drag"] = states["drag"][-1]

    # Integrate velocity and gamma
    state = rk4_step(state, flight_derivatives, current_state, dt)

    vertical_speed = state[0] * np.sin(state[1])
    altitude = current_state["altitude"] + vertical_speed * dt

    time = current_state["time"] + dt

    # Append states
    states["time"].append(time)
    states["velocity"].append(state[0])
    states["gamma"].append(state[1])
    states["beta"].append(state[2])
    states["altitude"].append(altitude)
    states["mach"].append(mach)
    states["alpha"].append(alpha)
    states["vertical_speed"].append(vertical_speed)
    states["distance"].append(distance)
    states["load_factor"].append(load_factor)
    states["x"].append(x)
    states["y"].append(y)

    states["air_density"].append(density)
    states["dyn_press"].append(dyn_press)
    states["cl"].append(clalpha_wing(np.rad2deg(alpha)))

    if current_state["altitude"] < 0:
        landed = True

print("\n##################################")

print(f"Flight duration: {round(states["time"][-1],1)}")
print(f"Vertical speed at touchdown: {current_state['vertical_speed']:.2f}")
print(f"Position at touchdown: {round(current_state['x']), round(current_state['y'])}\n")

print(f"Maximum lift: {max(states['lift']):.2f}")
print(f"Maximum drag: {max(states['drag']):.2f}")
print(f"Maximum wing moment: {min(states['wing_moment']):.2f}\n")

print(f"Maximum Mach number: {max(states['mach']):.2f}")
print(f"Maximum dynamic pressure: {max(states['dyn_press']):.2f}")

# print(f"Wing loading: {mass / S_wing}")

plot_flight_states(states)

export = False
if export:
    import pandas as pd

    save_directory = r"output database\Output for Sebas.csv"

    df = pd.DataFrame({
        'Velocity': export['velocity'],
        'Altitude': export['Altitude'],
        'Air density': export["Air density"],
        'C_L': export['C_L']
    })

    df.to_csv(save_directory, index=False)

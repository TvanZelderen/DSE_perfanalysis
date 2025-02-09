import numpy as np
from utils import get_isa, dynamic_pressure, mach_number
from math import pi
from plot import plot_flight_states
from velocity_controller import ImprovedVelocityController
from forces import get_forces
from airfoil import f_airfoil
from presets import *

from time import perf_counter

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



effective_wingspan = winglength * np.cos(sweep)
S_wing = effective_wingspan * wingchord_root

if S_wing == 0:
    Ar_wing = 0
else:
    Ar_wing = (effective_wingspan**2 / S_wing)

def update_sweep(sweep: int):
    sweep = np.deg2rad(sweep)
    effective_wingspan = winglength * np.cos(sweep)
    Ar_wing = (effective_wingspan**2 / S_wing)
    return sweep, Ar_wing

S_fin = finspan * (finchord_root + finchord_tip) / 2
if S_fin == 0:
    Ar_fin = 0
else:
    Ar_fin = finspan**2 / S_fin

clinterp, cdinterp, cminterp = f_airfoil(wing_airfoil, Ar = Ar_wing, e = e, wing = True)

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
    'l_over_d': [],
    'load_factor': [],
    'x': [],
    'y': [],
    'air_density': [],
    'moment': [],
    'dyn_press': [],
}

velocity_controller = ImprovedVelocityController(
    target_velocity=130,
    min_angle_of_attack=np.deg2rad(-7.5),
    max_angle_of_attack=np.deg2rad(10),
)

brakes_deployed = True
wings_deployed = False
wings_swept = True
landed = False
turn = True
spiral = False
landing_sequence = False

wing_airfoil = "kc135"
clinterp, cdinterp, cminterp = f_airfoil(wing_airfoil, Ar = Ar_wing, e = e, wing = False)

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
    states['wing_sweep_angle'].append(sweep)
    states['ar_wing'].append(Ar_wing)
    states['lift'].append(0)
    states['drag'].append(0)
    states['l_over_d'].append(0)
    states['alpha'].append(0)
    states['vertical_speed'].append(0)
    states['load_factor'].append(0)
    states['x'].append(distance)
    states['y'].append(0)
    states['air_density'].append(0)
    states['moment'].append(0)
    states['dyn_press'].append(0)

print("Starting sim...")
initialize_states()
print("Initialised states")
start = perf_counter()

shuttle_cock_dyn_press = 0


while not landed: 
    current_state = {key: value[-1] for key, value in states.items()}
    state = (current_state["velocity"], current_state["gamma"], current_state["beta"])

    pressure, density, temperature = get_isa(current_state["altitude"])
    dyn_press = dynamic_pressure(current_state["velocity"], current_state["altitude"])
    mach = mach_number(current_state["velocity"], current_state["altitude"])

    alpha = velocity_controller.update(current_state["velocity"], dt)

    if not wings_deployed and dyn_press > shuttle_cock_dyn_press:
        shuttle_cock_dyn_press = dyn_press
    
    if current_state['time'] > 5 and not wings_deployed:
        wings_deployed = True
        brakes_deployed = False
        print(f"Stabilized, deploying wings, {current_state['time']:.1f}s")
    if mach < 0.7 and mach < states['mach'][-1] and wings_swept and wings_deployed:
        sweep, Ar_wing = update_sweep(0)
        wings_swept = False
        print(f"Wings fully deployed, {current_state['time']:.1f}s")
        print(f"Velocity: {current_state['velocity']}, altitude: {current_state['altitude']}")
    if current_state["beta"] >= np.pi and turn: # Initial turn to 0 x distance
        turn = False
        print(f"Backtrack turn completed, {current_state['time']:.1f}s")
    if current_state['x'] <= 0 and not spiral and not landing_sequence: # Once at 0 x distance, start the spiral down
        spiral = True
        print(f"Spiral started, {current_state['time']:.1f}s")
    if current_state['altitude'] <= np.tan(landing_angle) * np.sqrt(current_state['x']**2 + current_state['y']**2) and not landing_sequence:
        landing_sequence = True
        print(f"Landing sequence started, {current_state['time']:.1f}s")
    if landing_sequence:
        target_angle = np.arctan2(-current_state['y'], -current_state['x'])
        alpha = np.deg2rad(8)
        spiral = False

    alpha_deg = np.rad2deg(alpha)
    lift, drag, moment, wing_moment = get_forces(current_state['altitude'], current_state["velocity"], alpha_deg, 0, clinterp, cdinterp, cminterp, brakes_deployed)

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
    
    load_factor = lift / current_state['weight']

    dx = current_state['velocity'] * np.cos(current_state['gamma']) * np.cos(current_state['beta']) * dt
    dy = current_state['velocity'] * np.cos(current_state['gamma']) * np.sin(current_state['beta']) * dt
    x = current_state['x'] + dx
    y = current_state['y'] + dy

    states["lift"].append(lift)
    states["drag"].append(drag)
    states["l_over_d"].append(lift/drag)
    states['moment'].append(wing_moment)

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

    if current_state["altitude"] < 0:
        landed = True

print("\n##################################")
print(f"Simulation time: {perf_counter() - start:.2f}\n")

print(f"Flight duration: {round(states['time'][-1],1)} s")
print(f"Vertical speed at touchdown: {current_state['vertical_speed']:.2f} m/s")
print(f"Horizontal speed at touchdown: {current_state['velocity'] * np.cos(current_state['gamma']):.2f} m/s")
print(f"Position at touchdown: {round(current_state['x']), round(current_state['y'])} m, m\n")

print(f"Maximum lift: {max(states['lift']):.2f} N")
print(f"Maximum drag: {max(states['drag']):.2f} N")
print(f"Maximum wing moment: {np.max(np.abs(states['moment'])):.2f} Nm")

print(f"Maximum Mach number: {max(states['mach']):.2f} -")
print(f"Maximum dynamic pressure: {max(states['dyn_press']):.2f} N/m^2")
print(f"Maximum shuttlecock dynamic pressure: {shuttle_cock_dyn_press:.2f} N/m^2")

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

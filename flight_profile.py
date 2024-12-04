from interpol_drag import launch_vehicle_drag_coef
import numpy as np
from utils import get_isa, dynamic_pressure, mach_number
from math import pi
from plot import plot_flight_states, plot_alpha
from velocity_controller import VelocityController
from airfoil import f_airfoil

e = 0.6
airfoil_wing = 'ah6407'
airfoil_fin = 'naca0012'

clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(airfoil_name = airfoil_wing) # alpha max =
clalpha_fin, cd0_fin, cmalpha_fin = f_airfoil(airfoil_name = airfoil_fin) # alpha max =

# print(clalpha_wing(5), clalpha_wing(10))
# print(clalpha_fin(5), clalpha_fin(10))

def flight_derivatives(state, params):
    """
    Calculate derivatives for longitudinal flight motion
    
    Parameters:
    state: [V, gamma] - current velocity and flight path angle
    params: Dictionary containing aircraft and environmental parameters
        - T: Thrust
        - alpha_T: Thrust angle
    """
    V, gamma = state
    
    # Unpack parameters
    L = params['lift']
    D = params['drag']
    W = params['weight']
    g = params['gravity']
    
    # Calculate derivatives using the given equations
    V_dot = (- D - W * np.sin(gamma)) / (W/g)
    gamma_dot = (L + - W * np.cos(gamma)) / (W/g * V)
    
    return np.array([V_dot, gamma_dot])

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
    state_1 = state_0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    
    return state_1

# Initial state
state = np.array([200.0, np.deg2rad(0)])  # [V, gamma]
time = 0 
dt = 0.01  # time step
landed = False

radius = 0.29 / 2
frontal_area = radius ** 2 * pi

wingspan = 1
wingchord = 0.12
S_wing = wingspan * wingchord
Ar_wing = wingspan ** 2 / S_wing

finspan = 0.1
fin_cr = 0.1
fin_ct = 0.1
S_fin = finspan * (fin_cr + fin_ct) / 2
Ar_fin = finspan ** 2 / S_fin

states = {
    'time': [],
    'velocity': [],
    'gamma': [],
    'altitude': [],
    'mach': [],
    'alpha': [],
}

params = {
        'weight': 20 * 9.81, # kg
        'gravity': 9.81,  # m/s^2
        'altitude': 27000, # m
        'alpha': np.deg2rad(5),
        'time': 0,
        'distance': 0,
        'vertical speed': 0,
    }

velocity_controller = VelocityController(
        target_velocity=100.0,  # m/s
        max_angle_of_attack=np.deg2rad(7)  # 15 degrees max AoA
    )



while not landed:
    params['time'] += dt

    pressure, density, temperature = get_isa(params['altitude'])
    dyn_press = dynamic_pressure(state[0], params['altitude'])
    mach = mach_number(state[0], params['altitude'])
    # print('mach', mach)

    params['alpha'] = velocity_controller.update(state[0], dt)

    cla_fin = float(clalpha_fin(params['alpha']))
    induced_drag_fins = dyn_press * (cd0_fin + cla_fin ** 2 / (np.pi * e * Ar_fin)) * S_fin
    cla_wing = float(clalpha_wing(params['alpha']))
    induced_drag_wing = dyn_press * (cd0_wing + cla_wing ** 2 / (np.pi * e * Ar_wing)) * S_wing
    drag_body = dyn_press * frontal_area * launch_vehicle_drag_coef(mach)
    params['drag'] = induced_drag_fins + induced_drag_wing + drag_body
    params['lift'] = dyn_press * clalpha_wing(params['alpha']) * S_wing

    state = rk4_step(state, flight_derivatives, params, dt)
    
    params['vertical speed'] = state[0] * np.sin(state[1])
    params['altitude'] += params['vertical speed'] * dt # V * sin(gamma)
    params['distance'] += state[0] * np.cos(state[1]) * dt # V * cos(gamma)

    states['time'].append(params['time'])
    states['velocity'].append(state[0])
    states['gamma'].append(np.rad2deg(state[1]))
    states['altitude'].append(params['altitude'])
    states['mach'].append(mach)
    states['alpha'].append(np.rad2deg(params['alpha']))

    if params['altitude'] < 3000:
        velocity_controller.target_velocity = 30

    if params['altitude'] < 0:
        landed = True

print(f"Vertical speed at touchdown: {params['vertical speed']}")
print(f"Distance travelled: {params['distance']}")
plot_flight_states(states)
plot_alpha(states)
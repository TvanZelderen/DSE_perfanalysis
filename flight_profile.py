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

clalpha_wing, cd0_wing, cmalpha_wing = f_airfoil(airfoil_name = airfoil_wing) # 2412 alpha max = 12 deg
clalpha_fin, cd0_fin, cmalpha_fin = f_airfoil(airfoil_name = airfoil_fin) # 0012 alpha max = 10.75

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

dt = 0.01  # time step

# Initial state

altitude = 27000
distance = 16000
velocity = 200

mass = 20
G = 9.81

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
    'vertical_speed': [],
    'distance': [],
    'lift': [],
    'drag': [],
    'turn_angle': [],
}

velocity_controller = VelocityController(
        target_velocity=100.0,  # m/s
        max_angle_of_attack=np.deg2rad(12)  # 15 degrees max AoA
    )

def initialize_states():
    # Set initial conditions
    states['time'].append(0)
    states['weight'] = [mass * G]
    states['gravity'] = [G]
    states['velocity'].append(velocity)
    states['mach'].append(mach_number(velocity, altitude))
    states['gamma'].append(0)
    states['altitude'].append(altitude)
    states['distance'].append(distance)
    states['lift'].append(0)
    states['drag'].append(0)
    states['alpha'].append(0)
    states['vertical_speed'].append(0)
    states['turn_angle'].append(0)


print(f"Starting sim...")
initialize_states()
print(f"Initialised states")

def initialize_states():
    # Set initial conditions
    states['time'].append(0)
    states['weight'] = [mass * G]
    states['gravity'] = [G]
    states['velocity'].append(velocity)
    states['mach'].append(mach_number(velocity, altitude))
    states['gamma'].append(0)
    states['altitude'].append(altitude)
    states['distance'].append(distance)
    states['lift'].append(0)
    states['drag'].append(0)
    states['alpha'].append(0)
    states['vertical_speed'].append(0)
    states['turn_angle'].append(0)


print(f"Starting sim...")
initialize_states()
print(f"Initialised states")

turn = True
bank_angle = np.radians(60)

while not landed:
    current_state = {key: value[-1] for key, value in states.items()}
    state = (current_state['velocity'], current_state['gamma'])


    ########### Turn Implementation ##########
    if 5 < current_state['time'] < 100 and turn:
        current_state['time'] += dt

    pressure, density, temperature = get_isa(current_state['altitude'])
    dyn_press = dynamic_pressure(current_state['velocity'], current_state['altitude'])
    mach = mach_number(current_state['velocity'], current_state['altitude'])

    alpha = velocity_controller.update(current_state['velocity'], dt)

    cla_fin = float(clalpha_fin(alpha))
    induced_drag_fins = dyn_press * (cd0_fin + cla_fin ** 2 / (np.pi * e * Ar_fin)) * S_fin
    cla_wing = float(clalpha_wing(alpha))
    induced_drag_wing = dyn_press * (cd0_wing + cla_wing ** 2 / (np.pi * e * Ar_wing)) * S_wing
    drag_body = dyn_press * frontal_area * launch_vehicle_drag_coef(mach)
    drag = induced_drag_fins + induced_drag_wing + drag_body
    lift = dyn_press * clalpha_wing(alpha) * wingspan * wingchord

    states['lift'].append(lift)
    states['drag'].append(drag)

    current_state['lift'] = states['lift'][-1]
    current_state['drag'] = states['drag'][-1]

    state = rk4_step(state, flight_derivatives, current_state, dt)

    time = current_state['time'] + dt
    
    ########### Turn Implementation ##########
    if current_state['turn_angle'] >= np.pi:
        turn = False
    if 5 < current_state['time'] and turn:
        horizontal_speed = current_state['velocity'] * np.cos(current_state['gamma'])
        delta_angle = current_state['gravity'] * np.tan(bank_angle) / current_state['velocity']
        turn_angle = current_state['turn_angle'] + delta_angle * dt
        states['turn_angle'].append(turn_angle)
        horizontal_new = horizontal_speed * np.cos(turn_angle)
        distance = distance = current_state['distance'] + horizontal_new * dt # V * cos(gamma)
    else:
        distance = current_state['distance'] + current_state['velocity'] * np.cos(state[1]) * dt # V * cos(gamma)
        states['turn_angle'].append(current_state['turn_angle'])


    vertical_speed = state[0] * np.sin(state[1])
    altitude = current_state['altitude'] + vertical_speed * dt # V * sin(gamma)
    

    # Append states
    states['time'].append(time)
    states['velocity'].append(state[0])
    states['gamma'].append(state[1])
    states['altitude'].append(altitude)
    states['mach'].append(mach)
    states['alpha'].append(alpha)
    states['vertical_speed'].append(vertical_speed)
    states['distance'].append(distance)

    if current_state['altitude'] < 3000:
        velocity_controller.target_velocity = 30

    if current_state['altitude'] < 0:
        landed = True

# print(states['gamma'])
print(f"Maximum velocity: {max(states['velocity'])} m/s")
print(f"Vertical speed at touchdown: {current_state['vertical_speed']}")
print(f"Distance travelled: {current_state['distance']}")
plot_flight_states(states)
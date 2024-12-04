from interpol_drag import launch_vehicle_drag_coef
import numpy as np
from utils import get_isa, dynamic_pressure, mach_number
from math import pi
from plot import plot_flight_states, plot_alpha
from velocity_controller import VelocityController
from airfoil import f_airfoil


bank_angle = np.radians(30) # 30 degrees in radians
# bank_acceleration = np.radians(30) # rad/s^2
bank_time = bank_angle / bank_acceleration
start = 0
g = 9.80665
dt = 0.01
ohm = 0

def turn(start, bank_angle = bank_angle, states, params):

    start = params['time']

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
    params['lift'] = dyn_press * clalpha_wing(params['alpha']) * S_wing * np.cos(bank_angle)

    state = rk4_step(state, flight_derivatives, params, dt)
    
    params['vertical speed'] = state[0] * np.sin(state[1])
    params['horizontal speed'] = state[0] * np.cos(state[1])
    params['turn angle'] = params['gravity'] * np.tan(bank_angle) / state[0]
    params['turn angle'] += params['turn angle'] * dt
    params['altitude'] += params['vertical speed'] * dt # V * sin(gamma)
    params['distance'] += state[0] * np.cos(state[1]) * dt # V * cos(gamma)


    states['time'].append(params['time'])
    states['velocity'].append(state[0])
    states['gamma'].append(np.rad2deg(state[1]))
    states['altitude'].append(params['altitude'])
    states['mach'].append(mach)
    states['alpha'].append(np.rad2deg(params['alpha']))

    start = start + dt

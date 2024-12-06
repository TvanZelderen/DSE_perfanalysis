from interpol_drag import launch_vehicle_drag_coef
import numpy as np
from utils import get_isa, dynamic_pressure, mach_number
from math import pi
from plot import plot_flight_states, plot_alpha
from airfoil import f_airfoil



class landingcontroller:
    def __init__(
        self,
        target_angle,
        min_angle_of_attack=0,
        max_angle_of_attack=0,
        Kp=0.05, # ideal value: 0.05
        Ki=0.004, # ideal value: 0.004
        Kd=0.1,
        max_integral_windup=1.0,
    ):
        """
        Initialize velocity feedback controller with PID control

        Parameters:
        - target_velocity: Desired velocity (m/s)
        - max_angle_of_attack: Maximum allowed angle of attack (radians)
        - Kp: Proportional gain
        - Ki: Integral gain
        - Kd: Derivative gain
        - max_integral_windup: Limit for integral term to prevent wind-up
        """
        self.target_angle = target_angle
        self.min_angle_of_attack = min_angle_of_attack
        self.max_angle_of_attack = max_angle_of_attack
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain

        # Controller state
        self.integral_error = 0
        self.previous_error = 0
        self.previous_angle_of_attack = 0
        self.max_integral_windup = max_integral_windup

    def update(self, current_angle, dt): 

        """
        Calculate the required angle of attack to approach target velocity

        Parameters:
        - current_velocity: Current velocity (m/s)
        - dt: Time step

        Returns:
        - Recommended angle of attack (radians)
        """
        # Calculate error
        error = self.target_angle - current_angle

        # Integral term with anti-windup
        self.integral_error += error * dt
        self.integral_error = max(
            -self.max_integral_windup,
            min(self.integral_error, self.max_integral_windup),
        )

        # Derivative term (rate of change of error)
        if dt > 0:
            derivative_error = (error - self.previous_error) / dt
        else:
            derivative_error = 0

        max_rate_of_change = 0.05  # example value in radians/second
        max_delta = max_rate_of_change * dt

        # PID controller calculation
        angle_of_attack = (
            self.Kp * error  # Proportional term
            + self.Ki * self.integral_error  # Integral term
            + self.Kd * derivative_error  # Derivative term
        )

        # Constrain angle of attack
        angle_of_attack = max(
            self.previous_angle_of_attack - max_delta,
            min(angle_of_attack, self.previous_angle_of_attack + max_delta)
        )

        angle_of_attack = max(
            self.min_angle_of_attack, min(angle_of_attack, self.max_angle_of_attack)
        )

        # Update previous error for next iteration
        self.previous_error = error
        self.previous_angle_of_attack = angle_of_attack

        return angle_of_attack



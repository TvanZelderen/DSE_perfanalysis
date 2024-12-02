from math import sqrt, exp, pi, atan2, cos, sin, tan, radians, acos

def body_to_earth_frame(body_frame_vector, orientation):
    pass

def earth_to_body_frame(earth_frame_vector, orientation):
    pass

def calculate_velocity_magnitude(vehicle):
    """Calculate the magnitude of the vehicle's velocity."""
    return sqrt(sum(v**2 for v in vehicle.velocity))

def calculate_velocity_direction(vehicle):
    """Calculate the direction of the vehicle's velocity vector."""
    velocity_magnitude = calculate_velocity_magnitude(vehicle)
    return [v / velocity_magnitude for v in vehicle.velocity] # Normalized velocity vector in x, y, and z

def calculate_velocity_pitch(vehicle):
    """Calculate the pitch angle of the vehicle's velocity vector."""
    horizontal_velocity_magnitude = sqrt(vehicle.velocity[0]**2 + vehicle.velocity[1]**2)
    return atan2(vehicle.velocity[2], horizontal_velocity_magnitude) # Angle around the lateral axis in radians

def calculate_velocity_yaw(vehicle):
    """Calculate the yaw angle of the vehicle's velocity vector."""
    return atan2(vehicle.velocity[1], vehicle.velocity[0]) # Angle around the vertical axis in radians

def calculate_drag(vehicle, air_density):
    """Calculate drag force."""
    velocity_magnitude = calculate_velocity_magnitude(vehicle)
    velocity_direction = calculate_velocity_direction(vehicle)
    drag_force = 0.5 * air_density * vehicle.drag_coefficient * vehicle.effective_drag_area() * velocity_magnitude**2
    # Drag direction is opposite to velocity
    drag_direction = [-v for v in velocity_direction]
    return [drag_force * d for d in drag_direction]

def calculate_lift_coefficient(vehicle, aoa):
    """Calculate lift coefficient using flat plate model"""
    if vehicle.wings_deployed:
        if - pi / 18 < aoa <= pi / 18: # 10 degrees
            return 2 * pi * aoa
        else: # Stall
            return 0
    return 0

def calculate_fin_force(vehicle, air_density):
    """Calculate fin force."""
    velocity_magnitude = sqrt(sum(v**2 for v in vehicle.velocity))
    fin_force = 0.5 * air_density * calculate_lift_coefficient() * vehicle.fin_area * velocity_magnitude**2

        
def calculate_lift(vehicle, air_density):
    """Calculate lift force."""
    velocity_magnitude = sqrt(sum(v**2 for v in vehicle.velocity))
    velocity_direction = calculate_velocity_direction(vehicle)
    aoa = vehicle.get_aoa()
    lift_coefficient = calculate_lift_coefficient(vehicle, aoa)
    lift_force = 0.5 * air_density * lift_coefficient * vehicle.wing_area * velocity_magnitude**2
    # Lift direction is perpendicular to velocity and AoA
    velocity_direction = [v / velocity_magnitude for v in vehicle.velocity] if velocity_magnitude != 0 else [0, 0, 0]

    return [lift_force * d for d in lift_direction]

def calculate_thrust(vehicle):
    """Calculate thrust force."""
    return [-vehicle.engine_thrust, 0, 0] if vehicle.thrust_enabled else [0, 0, 0]

def calculate_net_force(vehicle, gravity, air_density):
    """Calculate the net force acting on the vehicle."""
    drag = calculate_drag(vehicle, air_density)
    lift = [0, 0, 0] # calculate_lift(vehicle, air_density)
    thrust = calculate_thrust(vehicle)
    weight = [0, 0, -vehicle.mass * gravity]
    return [sum(f) for f in zip(drag, lift, thrust, weight)]

def calculate_aerodynamic_forces(vehicle):
    """
    Calculate aerodynamic forces on cylinder considering angle of attack.
    Returns forces in vehicle coordinates [x, y, z].
    """
    # Get flow properties
    velocity_magnitude = sqrt(sum(v**2 for v in vehicle.velocity))
    if velocity_magnitude < 0.1:  # Avoid division by zero
        return [0, 0, 0]
    
    air_density = vehicle.calculate_air_density()
    dynamic_pressure = 0.5 * air_density * velocity_magnitude**2
    
    # Calculate angle of attack
    horizontal_velocity = sqrt(vehicle.velocity[0]**2 + vehicle.velocity[1]**2)
    alpha = atan2(vehicle.velocity[2], horizontal_velocity)
    
    # Method 1: Full angle of attack consideration
    # Cross-sectional area varies with angle of attack
    projected_area = (vehicle.reference_area * cos(alpha) + 
                     pi * vehicle.reference_diameter * vehicle.reference_diameter * sin(abs(alpha)))
    
    # Normal force coefficient varies approximately with sin(2*alpha)
    Cn = 1.2 * sin(2 * alpha)  # 1.2 is approximate max Cn for cylinder
    
    # Axial force coefficient (mostly drag)
    Ca = -0.8  # Approximate drag coefficient for cylinder
    
    # Calculate forces in body coordinates
    normal_force = dynamic_pressure * projected_area * Cn
    axial_force = dynamic_pressure * vehicle.reference_area * Ca
    
    return [
        axial_force * cos(alpha) - normal_force * sin(alpha),  # X force
        0,                                                     # Y force (assuming symmetric flow)
        axial_force * sin(alpha) + normal_force * cos(alpha)   # Z force
    ]

def update_acceleration(vehicle, net_force):
    """Update the vehicle's acceleration."""
    vehicle.acceleration = [f / vehicle.mass for f in net_force]

def update_velocity(vehicle, time_step):
    """Update the vehicle's velocity."""
    vehicle.velocity = [v + a * time_step for v, a in zip(vehicle.velocity, vehicle.acceleration)]

def update_position(vehicle, time_step):
    """Update the vehicle's position using a second-order approximation."""
    vehicle.position = [
        p + v * time_step + 0.5 * a * (time_step ** 2)
        for p, v, a in zip(vehicle.position, vehicle.velocity, vehicle.acceleration)
    ]

def update_orientation(vehicle, time_step):
    """Update the vehicle's angular velocity and orientation."""
    vehicle.angular_velocity = [
        omega + alpha * time_step for omega, alpha in zip(vehicle.angular_velocity, vehicle.angular_acceleration)
    ]
    vehicle.orientation = [
        angle + omega * time_step for angle, omega in zip(vehicle.orientation, vehicle.angular_velocity)
    ]

def apply_rotational_physics(vehicle, time_step):
    """Apply rotational dynamics."""
    # aerodynamic_moments = vehicle.get_aerodynamic_moments()
    aerodynamic_moments = vehicle.compute_total_torque()
    vehicle.angular_acceleration = [
        moment / inertia for moment, inertia in zip(aerodynamic_moments, vehicle.moment_of_inertia)
    ]
    update_orientation(vehicle, time_step)

def demonstrate_fin_steering(vehicle):
    """
    This function demonstrates fin steering by changing the trajectory of the vehicle by adjusting the fin deflection.
    """
    vehicle.fin_deflection = pi / 6  # 30 degrees

def fin_steering_to_target(vehicle, target_position=[0, 0, 1000]):
    """
    Adjust fin deflection to steer the vehicle toward the target location.
    If fin steering is enabled, acquire the target and correct the fin deflection.
    """
    if vehicle.fin_deployed:
        target_direction = [t - p for t, p in zip(target_position, vehicle.position)]
        
        # Normalize the target direction to get a unit vector
        target_magnitude = sqrt(sum(t**2 for t in target_direction))
        if target_magnitude == 0:
            target_direction_normalized = [0, 0, 0]
        else:
            target_direction_normalized = [t / target_magnitude for t in target_direction]

        # Calculate the vehicle's current orientation (assuming we're only dealing with pitch for simplicity)
        # Calculate the vehicle's pitch angle in radians (assuming pitch is in orientation[0])
        vehicle_pitch = vehicle.orientation[0]

        # Calculate the pitch required to point toward the target
        target_pitch = atan2(target_direction_normalized[2], sqrt(target_direction_normalized[0]**2 + target_direction_normalized[1]**2))

        # Calculate the pitch error (difference between target pitch and current vehicle pitch)
        pitch_error = target_pitch - vehicle_pitch
        
        # Apply the pitch error to control the fin deflection
        # Limit the fin deflection to the maximum allowable value
        max_fin_deflection = vehicle.max_fin_deflection
        fin_deflection = max(-max_fin_deflection, min(pitch_error, max_fin_deflection))
        
        # Set the fin deflection to steer the vehicle
        vehicle.fin_deflection = fin_deflection

        # You can also adjust the yaw or roll steering in a similar way if needed.
        # For example, yaw control can be added by calculating a yaw error.

def simple_steer_to_target(vehicle, time_step=0.01):
    """
    Adjust the vehicle's velocity to steer toward the target position, 
    respecting the maximum turning force.

    Args:
        vehicle: The vehicle object.
        target_position (list): [x, y, z] coordinates of the target position.
        max_turning_acceleration (float): Maximum turning acceleration (m/s²).
    """
    target_position = [0, 0, vehicle.deployment_altitude]  

    # Calculate time to ground
    time_to_target_height = (target_position[2] - vehicle.position[2]) / (vehicle.velocity[2] if vehicle.velocity[2] != 0 else 0.1)
    gravity_loss = time_to_target_height * vehicle.velocity[2] * 0.5
    target_position = [target_position[0], target_position[1], target_position[2] + abs(gravity_loss)]

    if vehicle.position[2] < vehicle.deployment_altitude + 100:
        target_position[2] = 0

    # Calculate the target direction vector
    target_direction = [t - p for t, p in zip(target_position, vehicle.position)]
    target_distance = sqrt(sum(t**2 for t in target_direction))
    target_unit_vector = [t / target_distance for t in target_direction]

    # Calculate the current velocity direction
    velocity_magnitude = calculate_velocity_magnitude(vehicle)
    velocity_unit_vector = [v / velocity_magnitude for v in vehicle.velocity]

    # Determine the desired change in velocity (steering direction)
    steering_vector = [t - v for t, v in zip(target_unit_vector, velocity_unit_vector)]
    steering_magnitude = sqrt(sum(s**2 for s in steering_vector))

    # Normalize the steering vector and scale by max_turning_acceleration
    if steering_magnitude > 0:
        steering_unit_vector = [s / steering_magnitude for s in steering_vector]
    else:
        steering_unit_vector = [0, 0, 0]  # No steering needed if already aligned

    drag_force = calculate_drag(vehicle, vehicle.calculate_air_density())
    magnitude_drag_force = sqrt(sum(f**2 for f in drag_force))
    max_lift_force = vehicle.max_l_over_d * magnitude_drag_force

    max_turning_acceleration = max_lift_force / vehicle.mass


    steering_force = [
        s * max_turning_acceleration for s in steering_unit_vector
    ]

    # Update acceleration and velocity based on steering force
    vehicle.acceleration = [
        a + sf for a, sf in zip(vehicle.acceleration, steering_force)
    ]
    vehicle.velocity = [
        v + a * time_step for v, a in zip(vehicle.velocity, vehicle.acceleration)
    ]

def simulate_step(vehicle, time_step):
    """Perform one simulation step."""
    # Environmental calculations
    gravity = vehicle.calculate_gravity()
    air_density = vehicle.calculate_air_density()
    
    # Net force and acceleration
    net_force = calculate_net_force(vehicle, gravity, air_density)
    update_acceleration(vehicle, net_force)


    if vehicle.energy_turn and not vehicle.drogue_deployed:
        simple_steer_to_target(vehicle, time_step)
    
    # Update translational dynamics
    update_velocity(vehicle, time_step)
    update_position(vehicle, time_step)

    
    # Check landing conditions
    if vehicle.position[2] <= 0:
        vehicle.landed = True

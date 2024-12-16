import math
import cmath
import v_1.variables as var
from simulation.utils import get_air_density


def calculate_lift_coefficient(angle_of_attack):
    # Calculate lift coefficient based on angle of attack
    # Angle of attack is in radians
    return 2 * math.pi * angle_of_attack


def calculate_drag_coefficient(wing_area, reference_area):
    # Drag coefficient, 0.5 for spherical, 0.025 for crude aircraft
    if wing_area < 2 * reference_area:
        drag_coefficient = 0.5
    elif wing_area < 8 * reference_area:
        drag_coefficient = (wing_area / reference_area) * ((0.025 - 0.5) / 7) + 0.568
    else:
        drag_coefficient = 0.025


def calculate_lift_and_drag(
    velocity,
    air_density,
    angle_of_attack,
    parachute_deployed=False,
    wing_area=var.wing_area,
    reference_area=var.reference_area,
    parachute_area=var.parachute_area,
    parachute_cd=var.parachute_cd,
):
    velocity_magnitude = abs(velocity)
    velocity_angle = cmath.phase(velocity)

    # Calculate lift coefficient based on angle of attack, flat plate model
    lift_coefficient = calculate_lift_coefficient(angle_of_attack)
    drag_coefficient = calculate_drag_coefficient(wing_area, reference_area)

    # Calculate lift
    lift_force = (
        0.5
        * air_density
        * velocity_magnitude**2
        * wing_area
        * lift_coefficient
        * math.sin(angle_of_attack)
    )

    # Simple representation of stall
    if angle_of_attack > math.radians(16):
        lift_force = 0

    # Very crude model of lift for wing sizes under 8 * reference_area, 0 for wing sizes under 2 * reference_area
    if wing_area < 2 * reference_area:
        lift_force = 0
    elif wing_area < 8 * reference_area:
        lift_force = lift_force / 6 - 1 / 3

    # Lift is perpendicular to velocity
    lift_angle = velocity_angle + math.pi / 2
    lift_vector = lift_force * cmath.exp(1j * lift_angle)

    # Calculate drag (including parasitic and induced drag)
    induced_drag_factor = (lift_coefficient * angle_of_attack) ** 2 / (
        math.pi * (wing_area / reference_area)
    )
    total_cd = drag_coefficient + induced_drag_factor

    if parachute_deployed:
        total_cd += parachute_cd * (parachute_area / reference_area)

    drag_force = -0.5 * air_density * velocity_magnitude**2 * reference_area * total_cd
    drag_angle = velocity_angle - math.pi  # Drag opposes velocity
    drag_vector = drag_force * cmath.exp(1j * drag_angle)

    return lift_vector, drag_vector


def apply_engine_thrust(velocity_angle, engine_thrust=var.engine_thrust):
    # Engine thrust aligned with velocity vector
    return engine_thrust * cmath.exp(1j * velocity_angle)


def analyze_reachability(
    position, velocity, lift_coefficient, drag_coefficient, mass, G
):
    """
    Analyzes if the launch pad can be reached considering directional energy components
    and the ability to convert between them.

    Returns:
    - can_reach: boolean
    - required_turn_radius: float (minimum turn radius needed, or None if unreachable)
    - energy_margin: float (excess energy available for maneuvering)
    """
    # Split velocity into components
    v_horizontal = complex(velocity.real, 0)
    v_vertical = complex(0, velocity.imag)

    # Current energy state by component
    KE_horizontal = 0.5 * mass * abs(v_horizontal) ** 2
    KE_vertical = 0.5 * mass * abs(v_vertical) ** 2
    PE = mass * G * position.imag

    # Best glide characteristics
    best_glide_ratio = lift_coefficient / drag_coefficient * math.pi

    # Calculate energy required to reverse direction
    # This is the energy that will be lost to drag during the turn
    velocity_magnitude = abs(velocity)
    current_heading = math.atan2(velocity.imag, velocity.real)
    heading_to_pad = math.atan2(0 - position.imag, 0 - position.real)
    heading_change_needed = abs(
        (heading_to_pad - current_heading + math.pi) % (2 * math.pi) - math.pi
    )

    # Minimum turn radius at current speed
    # R = v²/(g * tan(bank_angle))
    max_bank_angle = math.pi / 3  # 60 degrees, typical maximum
    min_turn_radius = velocity_magnitude**2 / (G * math.tan(max_bank_angle))

    # Energy lost in turn (approximate)
    # E_lost = m*g*h_lost = m*g * (1-cos(heading_change_needed))*min_turn_radius
    energy_lost_in_turn = (
        mass * G * (1 - math.cos(heading_change_needed)) * min_turn_radius
    )

    # Energy required for final approach
    distance_to_pad = abs(position)
    min_approach_energy = mass * G * distance_to_pad / best_glide_ratio

    # Total energy required
    energy_required = energy_lost_in_turn + min_approach_energy

    # Available energy that can be converted to useful direction
    # Not all horizontal KE is useful if moving away from pad
    heading_efficiency = math.cos(heading_change_needed)
    useful_KE = KE_horizontal * max(0, heading_efficiency) + KE_vertical
    total_useful_energy = useful_KE + PE

    # Calculate energy margin
    energy_margin = total_useful_energy - energy_required

    # Check geometric constraints
    turn_diameter = 2 * min_turn_radius
    has_turning_space = position.imag > turn_diameter * 0.5  # Need height for turn

    # Results dictionary with detailed analysis
    results = {
        "can_reach": energy_margin > 0 and has_turning_space,
        "energy_margin": energy_margin,
        "min_turn_radius": min_turn_radius,
        "heading_change_needed": math.degrees(heading_change_needed),
        "required_height_for_turn": turn_diameter * 0.5,
        "current_height": position.imag,
        "best_glide_ratio": best_glide_ratio,
        "energy_components": {
            "KE_horizontal": KE_horizontal,
            "KE_vertical": KE_vertical,
            "PE": PE,
            "useful_KE": useful_KE,
        },
        "energy_requirements": {
            "turn_energy": energy_lost_in_turn,
            "approach_energy": min_approach_energy,
            "total_required": energy_required,
        },
    }
    return results["can_reach"]


def energy_state(position, velocity, mass, G):
    kinetic_energy = 0.5 * mass * abs(velocity) ** 2
    potential_energy = mass * G * position.imag
    total_energy = kinetic_energy + potential_energy
    return total_energy, kinetic_energy, potential_energy


def required_return_energy(
    position, velocity, mass, G, lift_coefficient, drag_coefficient
):
    # Required energy to

    # Energy required for final approach
    distance_to_pad = abs(position)
    min_approach_energy = mass * G * distance_to_pad / best_glide_ratio

    return min_approach_energy


def slope_normal_force_coefficient_fins(
    f=1,
    R=var.reference_diameter / 2,
    s=0.1,
    N=4,
    d=var.reference_diameter,
    l=0.1,
    cr=0.1,
    ct=0.1,
):
    """
    Source: https://www.nakka-rocketry.net/RD_fin.htm
    f: interference coefficient f=1 for 3 or 4 fins, f=0.5 for 6 fins
    R: rocket radius
    s: fin span
    N: number of fins
    d: reference diameter (nose cone diameter)
    l: mid chord length
    cr: root chord length
    ct: tip chord length
    """
    return (1 + (f * R) / (s + R)) * (
        (4 * N * (s / d) ** 2) / (1 + (1 + 2 * (l / (cr + ct)) ** 2) ** 0.5)
    )


def fin_force(aoa, position, velocity, A=var.reference_area):
    """
    Source: https://www.nakka-rocketry.net/RD_fin.htm
    q: dynamic pressure
    A: reference area
    V: velocity
    aoa: angle of attack
    """
    q = 0.5 * get_air_density(position.imag) * abs(velocity) ** 2
    return q * A * slope_normal_force_coefficient_fins(aoa)


def aerodynamic_force(aoa, position, velocity, A=var.reference_area, S=var.wing_area):
    S = 0.1 * 0.1 * 2  # two fins of 10 cm x 10 cm
    lift_coefficient = calculate_lift_coefficient(aoa)
    return (
        0.5 * get_air_density(position.imag) * abs(velocity) ** 2 * S * lift_coefficient
    )


def enhanced_turnback(
    position=(var.apogee_downrange + var.apogee_altitude * 1j),
    velocity=var.apogee_velocity,
    acceleration=var.apogee_acceleration,
    dt=var.dt,
    G=var.G,
    detachement_delay=var.detachement_delay,
    mass=10,  # kg
    thrust=False,
    print_state=False,
):
    time = 0

    # Flight phase flags
    detached = False
    turnback_completed = False
    touchdown = False
    parachute_deployed = False

    # Storage for trajectory data
    runtime = []
    position_trace = []
    velocity_trace = []
    acceleration_trace = []

    while not touchdown:
        # Time step update
        time += dt

        # Get current state
        altitude = position.imag
        velocity_angle = cmath.phase(velocity)
        air_density = get_air_density(altitude)

        # Calculate forces
        weight_force = -mass * G * 1j

        ### Phase-specific controls and forces
        # ------------------------------------
        # Phase 1: Initial descent
        if not detached:
            acceleration = -G * 1j
            if time >= detachement_delay:
                print(f"Detached at t = {time:.2f} s")
                detached = True

        elif not turnback_completed:
            # Phase 2: Active control phase
            angle_of_attack = 0.2  # radians (~11.5 degrees)
            lift_force, drag_force = calculate_lift_and_drag(
                velocity, air_density, angle_of_attack
            )
            if thrust:
                thrust_force = apply_engine_thrust(velocity_angle)
            else:
                thrust_force = 0

            # Sum all forces
            total_force = weight_force + lift_force + drag_force + thrust_force
            acceleration = total_force / mass

            # Check for turnback completion
            # TODO: Implement a more robust turnback completion check
            if math.pi / 2 > velocity_angle > -math.pi and position.real > 0:
                print(f"Turnback completed at t = {time:.2f} s")
                turnback_completed = True

        else:
            # Phase 3: Terminal descent
            if altitude < 1000 and not parachute_deployed:
                print(f"Deploying parachute at t = {time:.2f} s")
                parachute_deployed = True

            angle_of_attack = 0.1  # reduced angle for descent
            lift_force, drag_force = calculate_lift_and_drag(
                velocity, air_density, angle_of_attack
            )
            total_force = weight_force + lift_force + drag_force
            acceleration = total_force / mass

        # Update position and velocity
        position += velocity * dt + 0.5 * acceleration * dt**2
        velocity += acceleration * dt

        # Check for touchdown
        if position.imag <= 0:
            print(f"Touchdown at t = {time:.2f} s")
            touchdown = True
            position = position.real + 0j  # Set altitude to exactly 0

        # Record state
        runtime.append(time)
        position_trace.append(position)
        velocity_trace.append([velocity, cmath.phase(velocity)])
        acceleration_trace.append([acceleration, cmath.phase(acceleration)])

        # Print state if requested
        if print_state and time % 1 < dt:
            velocity_angle_deg = degrees(cmath.phase(velocity))
            acceleration_angle_deg = degrees(cmath.phase(acceleration))
            print(
                f"t = {time:.2f} s, x = {position.real:.2f} m, h = {position.imag:.2f} m"
            )
            print(f"v = {abs(velocity):.2f} m/s, angle = {velocity_angle_deg:.2f} deg")
            print(
                f"a = {abs(acceleration):.2f} m/s^2, angle = {acceleration_angle_deg:.2f} deg"
            )

    return runtime, position_trace, velocity_trace, acceleration_trace


if __name__ == "__main__":
    # print("Starting enhanced return calculation...")

    # time, position, velocity_trace, acceleration_trace = enhanced_turnback(
    #     print_state=True
    # )
    # print(f"Time to touchdown: {time:.2f} s")
    # print(f"Position at touchdown: {position.real:.2f} m downrange")

    # print("Return calculation completed.")

    position = 0 + 1000j
    velocity = 100 + 0j

    print(fin_force(0.2, position, velocity))
    print(aerodynamic_force(0.2, position, velocity))

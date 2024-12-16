import v_1.variables as var
import cmath
from math import degrees


def simple_turnback(
    position=(var.apogee_downrange + var.apogee_altitude * 1j),
    velocity=var.apogee_velocity,
    acceleration=var.apogee_acceleration,
    dt=var.dt,
    G=var.G,
    maximum_acceleration=var.maximum_acceleration,
    detachement_delay=var.detachement_delay,
    drag_coefficient=var.drag_coefficient,
    reference_area=var.reference_area,
    print_state=False,
):
    time = 0

    detached = False
    turnback_completed = False
    touchdown = False

    runtime = []
    position_trace = []
    velocity_trace = []
    acceleration_trace = []

    while not touchdown:
        # Update position, velocity, acceleration
        time += dt
        position += velocity * dt + 0.5 * acceleration * dt**2
        velocity += acceleration * dt

        # Calculate drag
        """
        Assume a sealevel air density of 1.225 kg/m^3
        """
        drag = -0.5 * 1.225 * velocity**2 * drag_coefficient * reference_area
        velocity_angle = cmath.phase(velocity)
        drag_angle = velocity_angle - cmath.pi
        drag = drag * cmath.exp(1j * drag_angle)
        acceleration = acceleration + drag

        # Phase 1: Detachement
        if not detached:
            acceleration = 0 - G * 1j  # Descend at 1G due to gravity
            if time >= detachement_delay:
                print("Detached at t = {:.2f} s".format(time))
                detached = True
        # Phase 2: Turn back
        elif not turnback_completed:
            """
            Turn back to the launch site
            A turn will be started, clockwise.
            The acceleration will be maximum_acceleration, at 90 degrees to the velocity vector.
            The turnback is completed when the velocity.real is -ve
            and the velocity.imag is 0.
            """
            velocity_angle = cmath.phase(velocity)
            acceleration_angle = velocity_angle - cmath.pi / 2
            acceleration = (
                maximum_acceleration * cmath.exp(1j * acceleration_angle) + -G * 1j
            )
            if (
                cmath.pi / 2 > velocity_angle > cmath.pi
            ):  # If the angle is greater than 180 degrees (pi radians)
                print("Turnback completed at t = {:.2f} s".format(time))
                turnback_completed = True
            if position.real < 0:
                print("Passed the launch site")
                touchdown = True
        # Phase 3: Driftdown
        else:
            """
            Drift down to the ground
            The acceleration will be 0 - Gj
            The drift down is completed when the altitude is 0
            """
            acceleration = 0 - G * 1j
        if position.imag <= 0:
            print("Touchdown at t = {:.2f} s".format(time))
            touchdown = True

        # Record the position and time
        runtime.append(time)
        position_trace.append(position)
        velocity_trace.append([velocity, cmath.phase(velocity)])
        acceleration_trace.append([acceleration, cmath.phase(acceleration)])

        # Every second, print the current state
        if print_state:
            if time % 1 < dt and time < 40:
                velocity_angle_deg = degrees(cmath.phase(velocity))
                acceleration_angle_deg = degrees(cmath.phase(acceleration))
                print(
                    f"t = {time:.2f} s, x = {position.real:.2f} m, h = {position.imag:.2f}"
                )
                print(f"v = {velocity:.2f} m/s, angle = {velocity_angle_deg:.2f} deg,")
                print(
                    f"a = {acceleration:.2f} m/s^2, angle = {acceleration_angle_deg:.2f} deg"
                )

    return runtime, position_trace, velocity_trace, acceleration_trace


if __name__ == "__main__":
    print("Starting return calculation...")

    time, position, velocity_trace, acceleration_trace = simple_turnback()
    print(f"Time to touchdown: {time:.2f} s")
    print(f"Position at touchdown: {position.real:.2f} m downrange")

    print("Return calculation completed.")

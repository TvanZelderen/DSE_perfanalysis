import matplotlib.pyplot as plt
from physics import simple_turnback
from physics2 import enhanced_turnback
from math import degrees


def get_color_gradient(num_points):
    color_stops = [
        (0, 0, 255),  # Blue
        (255, 0, 0),  # Red
        (255, 165, 0),  # Orange
    ]

    # Normalize all colors to 0-1 range
    color_stops = [(r / 255, g / 255, b / 255) for r, g, b in color_stops]

    # Calculate how many intervals we need between each color stop
    segments = len(color_stops) - 1
    points_per_segment = num_points // segments

    colors = []
    for i in range(segments):
        start_color = color_stops[i]
        end_color = color_stops[i + 1]

        # Calculate number of points for this segment
        # For the last segment, include any remaining points due to rounding
        if i == segments - 1:
            segment_points = num_points - len(colors)
        else:
            segment_points = points_per_segment

        # Calculate color intervals for this segment
        for j in range(segment_points):
            t = j / (segment_points - 1) if segment_points > 1 else 0
            r = start_color[0] + (end_color[0] - start_color[0]) * t
            g = start_color[1] + (end_color[1] - start_color[1]) * t
            b = start_color[2] + (end_color[2] - start_color[2]) * t
            colors.append((r, g, b))

    return colors


def plot_sim():
    """
    This function plots the trajectory of the return module.
    It plots the trajectory on the main plot.
    Subplots are used to plot the altitude, velocity (magnitude and angle), and acceleration (magnitude and angle) against time.
    """
    # Setup the plot
    fig, ax = plt.subplots(2, 2)

    # Draw a dot at the launch site
    ax[0, 0].plot(0, 0, "ro")

    sim_params = [10 * x for x in range(5)]

    colors = get_color_gradient(len(sim_params))

    for color_id, delay in enumerate(sim_params):
        # Run the simulation
        runtime, position, velocity_trace, acceleration_trace = simple_turnback(
            detachement_delay=delay
        )
        # runtime, position, velocity_trace, acceleration_trace = enhanced_turnback(
        #     detachement_delay=delay
        # )

        # Extract the x and y coordinates
        x = [p.real for p in position]
        y = [p.imag for p in position]

        # Extract the velocity and acceleration
        velocity = [abs(v[0]) for v in velocity_trace]
        velocity_angle = [degrees(v[1]) for v in velocity_trace]
        acceleration = [abs(a[0]) for a in acceleration_trace]
        acceleration_angle = [degrees(a[1]) for a in acceleration_trace]

        # Plot the trajectory
        ax[0, 0].plot(x, y)

        # Plot the altitude
        ax[0, 1].plot(runtime, y)

        # Get the color for this simulation
        color = colors[color_id]
        dark_color = (color[0] * 0.9, color[1] * 0.9, color[2] * 0.9)

        # Plot the velocity
        ax[1, 0].plot(runtime, velocity, label=f"vel {delay}", color=color)
        ax[1, 0].plot(runtime, velocity_angle, label=f"theta {delay}", color=dark_color)

        # Plot the acceleration
        ax[1, 1].plot(runtime, acceleration, label=f"accel {delay}", color=color)
        ax[1, 1].plot(
            runtime, acceleration_angle, label=f"theta {delay}", color=dark_color
        )

    # Set the labels
    ax[0, 0].set_title("Trajectory")
    ax[0, 0].set_xlabel("Downrange (m)")
    ax[0, 0].set_ylabel("Altitude (m)")

    ax[0, 1].set_title("Altitude")
    ax[0, 1].set_xlabel("Time (s)")
    ax[0, 1].set_ylabel("Altitude (m)")

    ax[1, 0].set_title("Velocity")
    ax[1, 0].set_xlabel("Time (s)")
    ax[1, 0].set_ylabel("Velocity (m/s)")
    ax2 = ax[1, 0].twinx()
    ax2.set_ylabel("Angle (deg)")
    ax2.set_ylim(-180, 180)

    ax[1, 1].set_title("Acceleration")
    ax[1, 1].set_xlabel("Time (s)")
    ax[1, 1].set_ylabel("Acceleration (m/s^2)")
    ax2 = ax[1, 1].twinx()
    ax2.set_ylabel("Angle (deg)")
    ax2.set_ylim(-180, 180)

    # Show the plot
    plt.show()


if __name__ == "__main__":
    plot_sim()

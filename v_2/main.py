from return_module import ReturnModule
from v_2.physics_alpha import simulate_step
from math import sqrt, degrees
import matplotlib.pyplot as plt

from v_2.physics_alpha import calculate_velocity_pitch, calculate_velocity_yaw

# Initialize vehicles
landers = [
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=0, energy_turn=False),
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=62.3, energy_turn=True),
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=62.3, energy_turn=True, fin_area=0.03),
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=111, energy_turn=True),
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=216, energy_turn=True),
    ReturnModule(altitude=27000, downrange=16000, velocity=200, detach_delay=10, engine_thrust=250, energy_turn=False),
]

# Simulation parameters
time_step = 0.01

# Data storage for plotting
simulation_data = []

def simulate_and_plot_landers(i, lander):
    print(f"Simulating lander {i+1}.")
    time = 0
    data = {
        "time": [],
        "altitude": [],
        "downrange": [],
        "velocity_magnitude": [],
        "acceleration_magnitude": [],
        "velocity_pitch": [],
        "velocity_yaw": [],
        "AoA": [],
        "drag_area": [],
    }

    delta_v = 0

    
    while not lander.landed:
        
        # Log data
        altitude = lander.position[2]
        downrange = sqrt(lander.position[0]**2 + lander.position[1]**2)
        velocity_magnitude = sqrt(sum(v**2 for v in lander.velocity))
        acceleration_magnitude = sqrt(sum(a**2 for a in lander.acceleration))/9.81 # Convert to g's
        velocity_pitch = degrees(calculate_velocity_pitch(lander))
        velocity_yaw = degrees(calculate_velocity_yaw(lander))
        drag_area = lander.effective_drag_area()
        
        data["time"].append(time)
        data["altitude"].append(altitude)
        data["downrange"].append(downrange)
        data["velocity_magnitude"].append(velocity_magnitude)
        data["acceleration_magnitude"].append(acceleration_magnitude)
        data["AoA"].append(lander.orientation[0])
        data["velocity_pitch"].append(velocity_pitch)
        data["velocity_yaw"].append(velocity_yaw)
        data["drag_area"].append(drag_area)

        delta_v += lander.engine_thrust / lander.mass * time_step


        # Detach from rocket
        if time >= lander.detach_delay:
            lander.detach()
            lander.thrust_enable()
            if downrange <= 100:
                lander.thrust_disable()
                delta_v += lander.velocity[0] + lander.velocity[1]
                lander.velocity[0] = 0
                lander.velocity[1] = 0
            # Deploy parachute
            if altitude <= lander.drogue_deployment_altitude and not lander.parachute_deployed:
                lander.deploy_drogue()
                lander.thrust_disable()
            if altitude <= lander.deployment_altitude and not lander.parachute_deployed:
                lander.deploy_parachute()
                lander.thrust_disable()
        
        # Update simulation
        simulate_step(lander, time_step)
        time += time_step
        
        # Check landing
        if altitude <= 0:
            lander.landed = True

    # if sqrt(lander.position[0]**2 + lander.position[1]**2) <= 500:
    print(f"Lander {i+1} has landed at {downrange:.2f} meters downrange, with a delta_v of {delta_v:.2f} || {delta_v*1.15:.2f}. Time: {time:.2f} seconds.")
        
    
    simulation_data.append(data)

for i, lander in enumerate(landers):
    simulate_and_plot_landers(i, lander)

# Initialize a figure with a custom layout
fig, axs = plt.subplot_mosaic([["big", "big", "tr"],["big", "big", "mr"],["bl", "bm", "br"],["bbl", "bbm", "bbr"]], figsize=(12, 12))

# Big plot: Altitude vs. Downrange
for i, data in enumerate(simulation_data):
    axs["big"].plot(data["downrange"], data["altitude"], label=f"Lander {i+1}")
axs["big"].scatter([0], [0], color='red', label="Launch Site", zorder=5)  # Add launch site dot
axs["big"].set_title("Altitude vs. Downrange")
axs["big"].set_xlabel("Downrange Distance (m)")
axs["big"].set_ylabel("Altitude (m)")
axs["big"].legend()
axs["big"].grid()

# Smaller plots
# Altitude vs. Time
for i, data in enumerate(simulation_data):
    axs["tr"].plot(data["time"], data["altitude"], label=f"Lander {i+1}")
axs["tr"].set_title("Altitude vs. Time")
axs["tr"].set_xlabel("Time (s)")
axs["tr"].set_ylabel("Altitude (m)")
# axs["tr"].legend()
axs["tr"].grid()

# Velocity vs. Time
for i, data in enumerate(simulation_data):
    axs["mr"].plot(data["time"], data["velocity_magnitude"], label=f"Lander {i+1}")
axs["mr"].set_title("Velocity Magnitude vs. Time")
axs["mr"].set_xlabel("Time (s)")
axs["mr"].set_ylabel("Velocity (m/s)")
# axs["mr"].legend()
axs["mr"].grid()

# Acceleration vs. Time
for i, data in enumerate(simulation_data):
    axs["br"].plot(data["time"], data["acceleration_magnitude"], label=f"Lander {i+1}")
axs["br"].set_title("Acceleration Magnitude vs. Time")
axs["br"].set_xlabel("Time (s)")
axs["br"].set_ylabel("Acceleration (g)")
# axs["br"].legend()
axs["br"].grid()

# Downrange vs. Time
for i, data in enumerate(simulation_data):
    axs["bbr"].plot(data["time"], data["downrange"], label=f"Lander {i+1}")
axs["bbr"].set_title("Downrange Distance vs. Time")
axs["bbr"].set_xlabel("Time (s)")
axs["bbr"].set_ylabel("Downrange Distance (m)")
# axs["bbr"].legend()
axs["bbr"].grid()

# # AoA vs. Time
# for i, data in enumerate(simulation_data):
#     axs["bm"].plot(data["time"], data["AoA"], label=f"Lander {i+1}")
# axs["bm"].set_title("Angle of Attack vs. Time")
# axs["bm"].set_xlabel("Time (s)")
# axs["bm"].set_ylabel("Angle of Attack (rad)")
# axs["bm"].legend()
# axs["bm"].grid()

# Velocity Pitch vs. Time
for i, data in enumerate(simulation_data):
    axs["bm"].plot(data["time"], data["velocity_pitch"], label=f"Lander {i+1}")
axs["bm"].set_title("Velocity Pitch vs. Time")
axs["bm"].set_xlabel("Time (s)")
axs["bm"].set_ylabel("Velocity Pitch (deg)")
# axs["bm"].legend()
axs["bm"].grid()

# # Pitch vs. Time
# for i, data in enumerate(simulation_data):
#     axs["bl"].plot(data["time"], data["pitch"], label=f"Lander {i+1}")
# axs["bl"].set_title("Pitch vs. Time")
# axs["bl"].set_xlabel("Time (s)")
# axs["bl"].set_ylabel("Pitch (rad)")
# axs["bl"].legend()
# axs["bl"].grid()

# Velocity Yaw vs. Time
for i, data in enumerate(simulation_data):
    axs["bl"].plot(data["time"], data["velocity_yaw"], label=f"Lander {i+1}")
axs["bl"].set_title("Velocity Yaw vs. Time")
axs["bl"].set_xlabel("Time (s)")
axs["bl"].set_ylabel("Velocity Yaw (deg)")
# axs["bl"].legend()
axs["bl"].grid()

# # Restoring Moment vs. Time
# for i, data in enumerate(simulation_data):
#     axs["bbl"].plot(data["time"], data["restoring_moment"], label=f"Lander {i+1}")
# axs["bbl"].set_title("Restoring Moment vs. Time")
# axs["bbl"].set_xlabel("Time (s)")
# axs["bbl"].set_ylabel("Restoring Moment (Nm)")
# axs["bbl"].legend()
# axs["bbl"].grid()

# Drag Area vs. Time
for i, data in enumerate(simulation_data):
    axs["bbl"].plot(data["time"], data["drag_area"], label=f"Lander {i+1}")
axs["bbl"].set_title("Drag Area vs. Time")
axs["bbl"].set_xlabel("Time (s)")
axs["bbl"].set_ylabel("Drag Area (m^2)")
# axs["bbl"].legend()
axs["bbl"].grid()

# # Fin Deflection vs. Time
# for i, data in enumerate(simulation_data):
#     axs["bm"].plot(data["time"], data["fin_deflection"], label=f"Lander {i+1}")
# axs["bm"].set_title("Fin Deflection vs. Time")
# axs["bm"].set_xlabel("Time (s)")
# axs["bm"].set_ylabel("Fin Deflection (rad)")
# axs["bm"].legend()
# axs["bm"].grid()

# # Aerodynamic Moment vs. Time
# for i, data in enumerate(simulation_data):
#     axs["bbm"].plot(data["time"], data["aerodynamic_moment"], label=f"Lander {i+1}")
# axs["bbm"].set_title("Aerodynamic Moment vs. Time")
# axs["bbm"].set_xlabel("Time (s)")
# axs["bbm"].set_ylabel("Aerodynamic Moment (Nm)")
# axs["bbm"].legend()
# axs["bbm"].grid()

# Adjust layout for better visibility
plt.tight_layout()

# Show the plots
plt.show()

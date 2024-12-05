import matplotlib.pyplot as plt
import numpy as np

def plot_flight_states(states):
    """
    Create a comprehensive plot of flight states.
    
    Parameters:
    states (dict): Dictionary containing time series data for flight parameters
    """
    # Create a figure with subplots
    fig, axs = plt.subplot_mosaic([["big", "big", "tr"],["big", "big", "mr"],["bl", "bm", "br"],["bbl", "bbm", "bbr"]], figsize=(12, 12), per_subplot_kw={"big": {'projection': '3d'}}) # ,["bbl", "bbm", "bbr"]
    fig.suptitle('Flight Simulation States', fontsize=16)

    x_data = states['x']
    y_data = states['y']
    z_data = states['altitude']

    axs["big"].plot3D(x_data, y_data, z_data)
    
    # Velocity plot
    axs["tr"].plot(states['time'], states['velocity'], color='blue')
    axs["tr"].set_title('Velocity vs Time')
    axs["tr"].set_xlabel('Time (s)')
    axs["tr"].set_ylabel('Velocity (m/s)')
    axs["tr"].grid(True)
    
    # Flight Path Angle plot
    gamma = [np.rad2deg(x) for x in states['gamma']]
    axs["bm"].plot(states['time'], gamma, color='green')
    axs["bm"].set_title('Flight Path Angle vs Time')
    axs["bm"].set_xlabel('Time (s)')
    axs["bm"].set_ylabel('Flight Path Angle (deg)')
    axs["bm"].grid(True)
    
    # Altitude plot
    axs["br"].plot(states['time'], states['altitude'], color='red')
    axs["br"].set_title('Altitude vs Time')
    axs["br"].set_xlabel('Time (s)')
    axs["br"].set_ylabel('Altitude (m)')
    axs["br"].grid(True)
    
    # Mach plot
    axs["bl"].plot(states['time'], states['mach'], color='purple')
    axs["bl"].set_title('Mach vs Time')
    axs["bl"].set_xlabel('Mach')
    axs["bl"].set_ylabel('Time (s)')
    axs["bl"].grid(True)

    # Alpha plot
    alpha = [np.rad2deg(x) for x in states['alpha']]
    axs["mr"].plot(states['time'], alpha)
    axs["mr"].set_title('Alpha vs Time')
    axs["mr"].set_xlabel('Time (s)')
    axs["mr"].set_ylabel('Alpha (deg)')
    axs["mr"].grid(True)

    # Turn plot
    bank = [np.rad2deg(x) for x in states['turn_angle']]
    axs["bbl"].plot(states['time'], bank)
    axs["bbl"].set_title('Turn vs Time')
    axs["bbl"].set_xlabel('Time (s)')
    axs["bbl"].set_ylabel('Phi (deg)')
    axs["bbl"].grid(True)

    # Turn plot
    axs["bbm"].plot(states['time'], states['lift'])
    axs["bbm"].set_title('Liftvs Time')
    axs["bbm"].set_xlabel('Time (s)')
    axs["bbm"].set_ylabel('Lift (N)')
    axs["bbm"].grid(True)

    # Turn plot
    axs["bbr"].plot(states['time'], states['drag'])
    axs["bbr"].set_title('Drag vs Time')
    axs["bbr"].set_xlabel('Time (s)')
    axs["bbr"].set_ylabel('Drag (N)')
    axs["bbr"].grid(True)
    
    # Adjust layout and display
    plt.tight_layout()
    plt.show()

# Optionally, add more specific plot functions
def plot_velocity(states):
    """Plot only velocity"""
    plt.figure(figsize=(10, 6))
    plt.plot(states['time'], states['velocity'])
    plt.title('Velocity vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.show()

def plot_altitude(states):
    """Plot only altitude"""
    plt.figure(figsize=(10, 6))
    plt.plot(states['time'], states['altitude'])
    plt.title('Altitude vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.grid(True)
    plt.show()

def plot_alpha(states):
    """Plot only altitude"""
    plt.figure(figsize=(10, 6))
    plt.plot(states['time'], states['alpha'])
    plt.title('Alpha vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Alpha (deg)')
    plt.grid(True)
    plt.show()
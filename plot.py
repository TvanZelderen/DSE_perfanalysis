import matplotlib.pyplot as plt

def plot_flight_states(states):
    """
    Create a comprehensive plot of flight states.
    
    Parameters:
    states (dict): Dictionary containing time series data for flight parameters
    """
    # Create a figure with subplots
    fig, axs = plt.subplot_mosaic([["big", "big", "tr"],["big", "big", "mr"],["bl", "bm", "br"]], figsize=(12, 12)) # ,["bbl", "bbm", "bbr"]
    fig.suptitle('Flight Simulation States', fontsize=16)
    
    # Velocity plot
    axs["tr"].plot(states['time'], states['velocity'], color='blue')
    axs["tr"].set_title('Velocity vs Time')
    axs["tr"].set_xlabel('Time (s)')
    axs["tr"].set_ylabel('Velocity (m/s)')
    axs["tr"].grid(True)
    
    # Flight Path Angle plot
    axs["bm"].plot(states['time'], states['gamma'], color='green')
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
    axs["mr"].plot(states['time'], states['alpha'])
    axs["mr"].set_title('Alpha vs Time')
    axs["mr"].set_xlabel('Time (s)')
    axs["mr"].set_ylabel('Alpha (deg)')
    axs["mr"].grid(True)
    
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
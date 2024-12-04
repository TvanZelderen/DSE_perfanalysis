import matplotlib.pyplot as plt

def plot_flight_states(states):
    """
    Create a comprehensive plot of flight states.
    
    Parameters:
    states (dict): Dictionary containing time series data for flight parameters
    """
    # Create a figure with subplots
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Flight Simulation States', fontsize=16)
    
    # Velocity plot
    axs[0, 0].plot(states['time'], states['velocity'], color='blue')
    axs[0, 0].set_title('Velocity vs Time')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Velocity (m/s)')
    axs[0, 0].grid(True)
    
    # Flight Path Angle plot
    axs[0, 1].plot(states['time'], states['gamma'], color='green')
    axs[0, 1].set_title('Flight Path Angle vs Time')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Flight Path Angle (deg)')
    axs[0, 1].grid(True)
    
    # Altitude plot
    axs[1, 0].plot(states['time'], states['altitude'], color='red')
    axs[1, 0].set_title('Altitude vs Time')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Altitude (m)')
    axs[1, 0].grid(True)
    
    # Velocity vs Altitude plot
    axs[1, 1].plot(states['time'], states['mach'], color='purple')
    axs[1, 1].set_title('Mach vs Time')
    axs[1, 1].set_xlabel('Mach')
    axs[1, 1].set_ylabel('Time (s)')
    axs[1, 1].grid(True)
    
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
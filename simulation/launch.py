from math import tan, radians

launch_angle = 78 # degrees
launch_angle_rad = radians(launch_angle) # radians
apogee_altitude = 26600 # meters
time_to_apogee = 81 # seconds
thrust_duration = 29 # seconds
G = 9.81 # m/s^2

def calculate_downrange_distance(
        launch_angle_rad=launch_angle_rad,
        apogee_altitude=apogee_altitude,
        time_to_apogee=time_to_apogee,
        G=G):
    # Calculate the virtual downrange distance
    downrange = apogee_altitude / tan(launch_angle_rad)

    # Calculate the gravity loss
    gravity_loss = 0.5 * G * time_to_apogee**2

    # Calculate the additional downrange distance
    additional_downrange = gravity_loss / tan(launch_angle_rad)

    # Calculate the total downrange distance
    total_downrange = downrange + additional_downrange

    return total_downrange

if __name__ == "__main__":
    print(f"Total downrange distance: {calculate_downrange_distance():.2f} meters")
import math

G = 9.81  # assumed to be constant

apogee_altitude = 21500  # meters
# apogee_downrange = 5000  # meters
apogee_downrange = apogee_altitude / math.tan(
    math.radians(85)
)  # meters, this is based on an 85 degree estimate from Miguel Castro
apogee_velocity = 14 + 0j  # m/s (28 m/s is about 100 km/h)

apogee_acceleration = 0 + 0j  # m/s^2

maximum_acceleration = 0.3 * G  # m/s^2

drag_coefficient = 0.5
reference_diameter = 0.25  # meters
reference_area = 0.25**2 * 3.14159  # meters^2
wing_area = 1.4 * reference_area  # m²
lift_coefficient = 1.2  # typical for simple wings

engine_thrust = 50  # N
thrust_vectoring = math.radians(10)  # radians

parachute_area = 10  # m²
parachute_cd = 1.75  # typical for round parachutes
mass = 5  # kg

detachement_delay = 30  # seconds

dt = 0.01  # seconds

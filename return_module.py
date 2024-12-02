from math import pi, atan2, sqrt


class ReturnModule:
    def __init__(
        self,
        altitude,
        downrange,
        velocity,
        mass=20,
        detach_delay=30,
        engine_thrust=0,
        wing_area=0.01,
        fin_area=0.02,
        c_l = 0.877,
        c_d = 0.5,
        reference_diameter=0.29,
        parachute_deployment_time=1,
        drogue_area=0.2,
        drogue_deployment_altitude=1500,
        parachute_area=0.8,
        parachute_deployment_altitude=500,
        energy_turn = False,
    ):
        # Translation properties
        self.position = [downrange, 0, altitude]  # x, y, z in meters
        self.velocity = [velocity, 0, 0]  # x, y, z in m/s
        self.acceleration = [0, 0, 0]  # x, y, z in m/s^2
        self.mass = mass  # kg
        self.reference_diameter = reference_diameter  # meters
        self.reference_area = (reference_diameter/2)**2 * pi  # meters^2

        # Rotation properties
        self.orientation = [0, 0]  # aoa, slip in radians
        self.angular_velocity = [0, 0]  # aoa, slip in rad/s
        self.angular_acceleration = [0, 0]  # aoa, slip in rad/s^2

        # Aerodynamic properties
        self.drag_coefficient = 0.5
        self.lift_coefficient = 0.0
        self.drogue_area = drogue_area  # m²
        self.parachute_area = parachute_area  # m²
        self.wing_area = wing_area # m²
        self.fin_area = fin_area # m²
        self.fin_arm = reference_diameter / 2  # meters
        self.fin_deflection = 0  # Fin angle in radians
        self.max_fin_deflection = pi / 6  # 30 degrees

        # Control properties
        self.engine_thrust = engine_thrust # N
        self.detach_delay = detach_delay  # seconds
        self.parachute_deployment_time = parachute_deployment_time # seconds
        self.drogue_deployment_altitude = drogue_deployment_altitude  # meters
        self.drogue_deployment_progress = 0
        self.deployment_altitude = parachute_deployment_altitude  # meters
        self.parachute_deployment_progress = 0

        self.energy_turn = energy_turn
        self.max_l_over_d = (c_l * fin_area) / (c_d * self.reference_area)

        self.detached = False
        self.drogue_deployed = False
        self.parachute_deployed = False
        self.fin_deployed = False
        self.wings_deployed = False
        self.thrust_enabled = False
        self.landed = False

    def detach(self):
        self.detached = True

    def deploy_drogue(self):
        if not self.drogue_deployed:
            self.drogue_deployed = True

    def detach_drogue(self):
        if self.drogue_deployed:
            self.drogue_deployed = False

    def deploy_parachute(self):
        if not self.parachute_deployed:
            self.parachute_deployed = True

    def detach_parachute(self):
        if self.parachute_deployed:
            self.parachute_deployed = False

    def effective_drag_area(self):
        if self.drogue_deployed and not self.parachute_deployed:
            parachute_area = self.reference_area * (1 - self.drogue_deployment_progress) + self.drogue_area * self.drogue_deployment_progress
            if self.drogue_deployment_progress < self.parachute_deployment_time:
                self.drogue_deployment_progress += 0.01
            return parachute_area
        if self.parachute_deployed:
            parachute_area = self.reference_area * (1 - self.drogue_deployment_progress) + (self.drogue_area if self.drogue_deployed else 0) * (1 - self.drogue_deployment_progress) + self.parachute_area * self.parachute_deployment_progress
            if self.parachute_deployment_progress < self.parachute_deployment_time:
                self.parachute_deployment_progress += 0.01
            return parachute_area
        return self.reference_area

    def fin_steering_enabled(self):
        self.fin_deployed = True

    def fin_steering_disabled(self):
        self.fin_deployed = False

    def update(self, time_step):
        self.update_translational_dynamics(time_step)
        self.update_rotational_dynamics(time_step)

    def get_aoa(self):
        """
        Calculate the angle of attack (AoA) of the vehicle.
        AoA is the angle between the velocity vector and the pitch direction of the vehicle.
        """
        horizontal_velocity_magnitude = sqrt(self.velocity[0]**2 + self.velocity[1]**2)
        velocity_pitch = atan2(self.velocity[2], horizontal_velocity_magnitude)
        vehicle_pitch = self.orientation[0]

        # Calculate and return the AoA
        return velocity_pitch - vehicle_pitch

    def align_with_velocity(vehicle):
        """Instantly align the vehicle's orientation with its velocity vector."""
        vx, vy, vz = vehicle.velocity
        velocity_magnitude = sqrt(vx**2 + vy**2 + vz**2)

        if velocity_magnitude == 0:
            # If velocity is zero, retain the current orientation (no meaningful alignment possible).
            return

        # Calculate pitch and yaw angles from velocity vector.
        pitch = atan2(vz, sqrt(vx**2 + vy**2))  # Angle relative to horizontal plane.
        yaw = atan2(vy, vx)  # Angle around the vertical axis.

        # Update the vehicle's orientation to match the velocity vector.
        vehicle.orientation = [pitch, yaw, 0]  # Roll is left unchanged.


    def compute_velocity_orientation(self):
        """Calculate the pitch and yaw angles based on the velocity vector."""
        vx, vy, vz = self.velocity

        pitch = atan2(vz, sqrt(vx**2 + vy**2))  # Vertical angle
        yaw = atan2(vy, vx)  # Horizontal angle
        return [pitch, yaw, 0]  # No roll alignment needed for now
    
    def compute_restoring_torque(self, stiffness=1.0):
        """Compute restoring torque to align orientation with velocity vector."""
        desired_orientation = self.compute_velocity_orientation()
        current_orientation = self.orientation
        
        # Difference between desired and current orientation
        orientation_error = [
            desired - current
            for desired, current in zip(desired_orientation, current_orientation)
        ]
        
        # Restoring torque proportional to orientation error
        restoring_torque = [-stiffness * error for error in orientation_error]
        return restoring_torque
    
    def compute_total_torque(self):
        """Compute total torque acting on the vehicle."""
        restoring_torque = self.compute_restoring_torque()
        
        # Example: Control torques from fins or angle of attack
        control_torque = self.get_aerodynamic_moments()  # This uses the fins and AoA
        
        # Combine torques
        total_torque = [restore + control for restore, control in zip(restoring_torque, control_torque)]
        return total_torque

    def get_aerodynamic_moments(self):
        dynamic_pressure = (
            0.5 * self.calculate_air_density() * sum(v**2 for v in self.velocity)
        )
        roll_moment = 0.1 * self.fin_deflection * dynamic_pressure
        pitch_moment = 0.1 * self.fin_deflection * dynamic_pressure
        yaw_moment = 0.02 * self.fin_deflection * dynamic_pressure
        return [pitch_moment, yaw_moment, roll_moment]

    def wing_deployment(self):
        self.lift_coefficient = 1.2
        self.wings_deployed = True

    def thrust_enable(self):
        self.thrust_enabled = True

    def thrust_disable(self):
        self.thrust_enabled = False

    def landing(self):
        self.landed = True

    ### Environmental properties ###
    def calculate_gravity(self):
        # Simplified gravity model
        earth_radius = 6371e3  # Earth radius in meters
        return 9.80665 * (earth_radius / (earth_radius + self.position[2])) ** 2

    def calculate_air_density(self):
        # Simplified ISA model for air density
        # Truly terrible model, but better than nothing.
        sea_level_density = 1.225  # kg/m³
        scale_height = 8500  # meters
        return sea_level_density * (2.718) ** (-self.position[2] / scale_height)

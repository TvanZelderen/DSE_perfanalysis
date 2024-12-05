class ImprovedVelocityController:
    def __init__(
        self,
        target_velocity,
        min_angle_of_attack=0,
        max_angle_of_attack=0,
        Kp=0.01,
        Ki=0.0001,
        Kd=0.005,
        max_integral_windup=1.0,
    ):
        """
        Initialize velocity feedback controller with PID control

        Parameters:
        - target_velocity: Desired velocity (m/s)
        - max_angle_of_attack: Maximum allowed angle of attack (radians)
        - Kp: Proportional gain
        - Ki: Integral gain
        - Kd: Derivative gain
        - max_integral_windup: Limit for integral term to prevent wind-up
        """
        self.target_velocity = target_velocity
        self.min_angle_of_attack = min_angle_of_attack
        self.max_angle_of_attack = max_angle_of_attack
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain

        # Controller state
        self.integral_error = 0
        self.previous_error = 0
        self.max_integral_windup = max_integral_windup

    def update(self, current_velocity, dt):
        """
        Calculate the required angle of attack to approach target velocity

        Parameters:
        - current_velocity: Current velocity (m/s)
        - dt: Time step

        Returns:
        - Recommended angle of attack (radians)
        """
        # Calculate error
        error = self.target_velocity - current_velocity

        # Integral term with anti-windup
        self.integral_error += error * dt
        self.integral_error = max(
            -self.max_integral_windup,
            min(self.integral_error, self.max_integral_windup),
        )

        # Derivative term (rate of change of error)
        if dt > 0:
            derivative_error = (error - self.previous_error) / dt
        else:
            derivative_error = 0

        # PID controller calculation
        angle_of_attack = -(
            self.Kp * error  # Proportional term
            + self.Ki * self.integral_error  # Integral term
            + self.Kd * derivative_error  # Derivative term
        )

        # Constrain angle of attack
        angle_of_attack = max(
            self.min_angle_of_attack, min(angle_of_attack, self.max_angle_of_attack)
        )

        # Update previous error for next iteration
        self.previous_error = error

        return angle_of_attack

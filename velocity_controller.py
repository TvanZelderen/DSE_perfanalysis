class VelocityController:
    def __init__(self, target_velocity, min_angle_of_attack=-7.5, max_angle_of_attack=14, 
                 Kp=0.1, Ki=0.01, max_integral_windup=1.0):
        """
        Initialize velocity feedback controller
        
        Parameters:
        - target_velocity: Desired velocity (m/s)
        - max_angle_of_attack: Maximum allowed angle of attack (radians)
        - Kp: Proportional gain
        - Ki: Integral gain
        - max_integral_windup: Limit for integral term to prevent wind-up
        """
        self.target_velocity = target_velocity
        self.min_angle_of_attack = min_angle_of_attack
        self.max_angle_of_attack = max_angle_of_attack
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        
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
        self.integral_error = max(-self.max_integral_windup, 
                                  min(self.integral_error, self.max_integral_windup))
        
        # PI controller calculation
        angle_of_attack = (
            self.Kp * error +  # Proportional term
            self.Ki * self.integral_error  # Integral term
        )
        
        # Constrain angle of attack
        angle_of_attack = max(-self.max_angle_of_attack, 
                               min(angle_of_attack, self.max_angle_of_attack))
        
        return angle_of_attack

# File: utilities/pid_controller.py
class PIDController:
    def __init__(self, kp, ki, kd):
        """
        Initialize the PID controller with the given coefficients.
        """
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain

        # Internal state
        self.prev_error = 0.0
        self.error_integral = 0.0
        self.prev_time = None

    def reset(self):
        """
        Reset the PID controller's internal state.
        """
        self.prev_error = 0.0
        self.error_integral = 0.0
        self.prev_time = None

    def compute(self, current_value, target_value, current_time):
        if not self.prev_time:
            self.prev_time = current_time
            return 0
        # Calculate error
        error = target_value - current_value
        # Calculate time delta
        delta_time = current_time - self.prev_time
            
        # Update internal state
        self.prev_time = current_time

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.error_integral += error * delta_time
        integral = self.ki * self.error_integral

        # Derivative term
        derivative = self.kd * (error - self.prev_error) / delta_time

        # Control signal
        control_signal = proportional + integral + derivative

        # Update previous error
        self.prev_error = error

        return control_signal

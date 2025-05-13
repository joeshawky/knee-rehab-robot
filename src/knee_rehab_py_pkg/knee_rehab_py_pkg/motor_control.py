#!/usr/bin/env python3
from math import fabs
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from knee_rehab_interfaces.msg import AngleTimeStamped
from knee_rehab_interfaces.msg import BatteryStatusTimeStamped 
from knee_rehab_interfaces.msg import ModeStatus
from knee_rehab_interfaces.msg import ForceTimeStamped
from knee_rehab_py_pkg.utilities.knee_rehab_modes import KneeRehabModes
from knee_rehab_py_pkg.utilities.pid_controller import PIDController
from rclpy.impl.rcutils_logger import RcutilsLogger

# import RPi.GPIO as GPIO


class MotorControlNode(Node): 
    def __init__(self):
        super().__init__("motor_control")
        self.init_parameters()
        self.init_variables()
        self.init_publishers()
        self.init_subscriptions()

        self.get_logger().info("Motor control node has been initialised.")


    def init_parameters(self):
        self.declare_parameter("target_angle_publisher_timer", 1.0)
        self.declare_parameter("pid_exercise_timer", 1.0)
        self.declare_parameter("pid_training_timer", 1.0)
        self.declare_parameter("pid_free_timer", 1.0)
        self.declare_parameter("pid_kp", 1.0)
        self.declare_parameter("pid_ki", 1.0)
        self.declare_parameter("pid_kd", 1.0)


    def init_variables(self):
        self.logger: RcutilsLogger = self.get_logger()
        self.target_angle_publisher_timer = self.get_parameter("target_angle_publisher_timer").value
        self.pid_exercise_timer = self.get_parameter("pid_exercise_timer").value
        self.pid_training_timer = self.get_parameter("pid_training_timer").value
        self.pid_free_timer = self.get_parameter("pid_free_timer").value
        self.pid_kp = self.get_parameter("pid_kp").value
        self.pid_ki = self.get_parameter("pid_ki").value
        self.pid_kd = self.get_parameter("pid_kd").value
        self.pid_exercise = PIDController(self.pid_kp, self.pid_ki, self.pid_kd)
        self.pid_training = PIDController(self.pid_kp, self.pid_ki, self.pid_kd)
        self.force = 0

        self.training_timer = None
        self.exercise_timer = None
        self.free_timer = None

        # self.init_gpio()
        
        self.init_training_mode_variables()
        
    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        self.dir_pin = 18 # REPLACE WITH DIR PIN!
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.output(self.dir_pin, GPIO.LOW) # LOW = CW, HIGH = CCW

        self.pwm_pin = 19 # REPLACE WITH PWM PIN!
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)
        self.pwm.start(0)


    def init_training_mode_variables(self):
        self.target_angles: list[tuple[float, Time]] = []
        self.is_recording = False
        self.last_direction = None
        self.last_angle = None
        self.current_angle = None
        self.current_angle_time = None
        self.last_mode_status = "FREEMODE MODE_STOP"


    def init_publishers(self):
        self.target_angle_publisher = self.create_publisher(AngleTimeStamped, "target_angle", 10)
        self.create_timer(self.target_angle_publisher_timer, self.callback_publish_target_angle)
    
    def callback_publish_target_angle(self):
        msg = AngleTimeStamped()
        msg.angle = -1.0
        msg.time.sec = -1
        msg.time.nanosec = 0

        if len(self.target_angles) != 0:
            msg.angle = self.target_angles[0][0]
            msg.time = self.target_angles[0][1]

        self.target_angle_publisher.publish(msg)

    def init_subscriptions(self):
        self.create_subscription(BatteryStatusTimeStamped, "ina219_data", self.callback_ina219_data, 10)
        self.create_subscription(AngleTimeStamped, "as5600_data", self.callback_as5600_data, 10)
        self.create_subscription(ForceTimeStamped, "ads1115_data", self.callback_ads1115_data, 10)
        self.create_subscription(ModeStatus, "current_mode", self.callback_current_mode, 10)


    def callback_ina219_data(self, msg: BatteryStatusTimeStamped):
        voltage = msg.voltage
        current = msg.current
        msg_time = msg.time


    def callback_as5600_data(self, msg: AngleTimeStamped):
        self.current_angle = msg.angle
        self.current_angle_time = msg.time
        if self.is_recording:
            self.capture_peak_points(msg.angle, msg.time)


    def callback_ads1115_data(self, msg: ForceTimeStamped):
        self.force = msg.force


    def capture_peak_points(self, angle, timestamp):
        # Determine direction of angle change
        if self.last_angle is not None:
            current_direction = 'increasing' if angle > self.last_angle else 'decreasing'
            
            # Detect a peak or valley if the direction changes
            if self.last_direction and current_direction != self.last_direction:
                self.target_angles.append((self.last_angle, self.last_angle_timestamp))
                self.logger.info(f"Captured peak point: angle={self.last_angle}, time={self.last_angle_timestamp}")
            
            # Update direction and last angle information
            self.last_direction = current_direction

        # Update the last angle and timestamp for the next comparison
        self.last_angle = angle
        self.last_angle_timestamp = timestamp

    
    def callback_current_mode(self, msg: ModeStatus):
        mode = KneeRehabModes.get_mode_name(msg.mode) 
        mode_status = KneeRehabModes.get_status_name(msg.is_active)

        mode_start = mode_status == "MODE_START"
        mode_stop = mode_status == "MODE_STOP"

        # self.logger.info(f"is_recording: {self.is_recording}")
        self.logger.info(f"{mode} {mode_status}")
        
        if self.last_mode_status == f"{mode} {mode_status}":
            return

        self.last_mode_status = f"{mode} {mode_status}"
        match(mode):
            case "EXERCISE_MODE":
                target_angles_available = len(self.target_angles) != 0
                if mode_start and target_angles_available:
                    self.start_exercise_mode()
                elif mode_stop:
                    self.last_mode_status = "EXERCISE_MODE MODE_START"
                    self.stop_exercise_mode()
            case "TRAINING_MODE":
                if mode_start:
                    self.start_training_mode()
                elif mode_stop:
                    self.stop_training_mode()
            case "FREE_MODE":
                if mode_start:
                    self.start_free_mode()
                elif mode_stop:
                    self.stop_free_mode()
        
                
                
    def start_exercise_mode(self):
        self.logger.info("Started exercise mode.")
        self.is_recording = False
        if not self.target_angles:
            self.logger.info("No target angles available.")
            return
        
        self.pid_exercise.reset()
        if self.exercise_timer is None:
            self.exercise_timer = self.create_timer(self.pid_exercise_timer, self.exercise_mode_callback)


    def stop_exercise_mode(self):
        if self.exercise_timer is not None:
            self.exercise_timer.cancel()
            self.exercise_timer = None
            self.logger.info("Exercise mode timer stopped.")

    def start_training_mode(self):
        self.is_recording = True
        self.target_angles.clear()
        self.target_angles.append((self.current_angle, self.current_angle_time))  
        self.last_angle = self.current_angle
        self.last_direction = None
        self.logger.info("Started recording in TRAINING_MODE with initial angle.")
        if self.exercise_timer:
            self.exercise_timer.cancel()

        self.training_timer = self.create_timer(self.pid_training_timer, self.training_mode_callback)  # 10 ms timer


    def stop_training_mode(self):
        self.is_recording = False
        # Append the last angle to capture the end state
        if self.current_angle and self.current_angle_time:
            self.target_angles.append((self.current_angle, self.current_angle_time))

        self.logger.info("Stopped recording in TRAINING_MODE and added final angle.")
        self.last_angle = None
        self.last_direction = None

        if self.exercise_timer:
            self.exercise_timer.cancel()
        
        if self.training_timer:
            self.training_timer.cancel()


    def training_mode_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Use the latest force value updated by the callback
        current_force = self.force
        target_force = 5  # Adjust this based on your requirements

        # Compute control signal
        control_signal = self.pid_training.compute(current_force, target_force, current_time, self.logger)

        self.logger.info(f"Target Force: {target_force:.2f}, Current Force: {current_force:.2f}, Control: {control_signal:.2f}")

        

        pwm_value = int(fabs(control_signal))

        if pwm_value > 255:
            pwm_value = 255


        if control_signal < 0:
            # go CCW
            self.set_motor_dir(-1)
        elif control_signal > 0:
            # go CW
            self.set_motor_dir(1)
        
        
        # Update motor with control signal
        self.set_motor_pwm(control_signal)


    
    def exercise_mode_callback(self):
        if not self.target_angles:
            self.stop_exercise_mode()
            return

        # Take the first target angle
        target_angle, _ = self.target_angles[0]

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Compute control signal
        control_signal = self.pid_exercise.compute(self.current_angle, target_angle, current_time)

        # Clamp control signal to motor limits
        control_signal = max(min(control_signal, 255), 0)

        # Update motor with control signal
        self.set_motor_pwm(control_signal)

        self.logger.info(f"Target angle: {target_angle:.2f}, Current angle: {self.current_angle:.2f}, Control: {control_signal:.2f}")

        # Check if target angle is reached
        if abs(self.current_angle - target_angle) <= 1.0:  # 1-degree tolerance
            self.logger.info(f"Reached target angle: {target_angle:.2f}")
            self.target_angles.pop(0)  # Remove the completed target
            if not self.target_angles:  # Stop timer if all targets are reached
                self.stop_exercise_mode()
        

    def start_free_mode(self):
        '''
        Initiate PID control with force sensors
        '''
        self.is_recording = False
        self.free_timer = self.create_timer(self.pid_free_timer, self.training_mode_callback) 
        # This mode is the same as training but without recording data.

    def stop_free_mode(self):
        '''
        Stop PID control with force sensors
        '''
        if self.free_timer:
            self.free_timer.cancel()
        
        if self.training_timer:
            self.training_timer.cancel()
        
        if self.exercise_timer:
            self.exercise_timer.cancel()

        

    def set_motor_pwm(self, control_signal):
        # control signal is 0-255
        # modified pwm value is 0.0-1.0
        modified_pwm_value = control_signal / 255.0
        # self.pwm.ChangeDutyCycle(modified_pwm_value)
        ...


    def set_motor_dir(self, dir):
        #1  = CW
        #-1 = CCW
        # value = GPIO.HIGH if dir == -1 else GPIO.LOW
        # GPIO.output(self.dir_pin, value)
        ...

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

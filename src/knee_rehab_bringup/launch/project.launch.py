from launch import LaunchDescription
from launch_ros.actions import Node

def init_real(package_name="knee_rehab_py_pkg"):
    ld = LaunchDescription()

    mode_publisher_timer = 5.0
    client_commands_send_timer = 0.2
    current_angle_publisher_timer = 0.2
    motor_power_publisher_timer = 5.0
    battery_status_publisher_timer = 5.0
    target_angle_publisher_timer = 5.0




    bluetooth_manager_node = Node(
        package=package_name,
        executable="bluetooth_manager",
        parameters=[
            {
                "mode_publisher_timer": mode_publisher_timer,
                "client_commands_send_timer": client_commands_send_timer,
            }
        ]
    )

    magnetic_encoder_node = Node(
        package=package_name,
        executable="magnetic_encoder",
        parameters=[
            {
                "current_angle_publisher_timer": current_angle_publisher_timer
            }
        ]
    )

    power_sensing_node = Node(
        package=package_name,
        executable="power_sensing",
        parameters= [
            {
                "motor_power_publisher_timer": motor_power_publisher_timer,
                "battery_status_publisher_timer": battery_status_publisher_timer
            }
        ]
    )

    motor_control_node = Node(
        package=package_name,
        executable="motor_control",
        parameters=[
            {
                "target_angle_publisher_timer": target_angle_publisher_timer
            }
        ]
    )

    ld.add_action(bluetooth_manager_node)
    ld.add_action(magnetic_encoder_node)
    ld.add_action(power_sensing_node)
    ld.add_action(motor_control_node)


def init_simulation(ld, package_name):
    package_name = "knee_rehab_py_pkg"



    

    as5600_publisher_timer = 0.1
    ina219_publisher_timer = 0.1
    ads1115_publisher_timer = 0.1

    min_value = 1.0
    max_value = 50.0
    counter = 0.1


    target_angle_publisher_timer = 1.0
    pid_timer = 0.1
    kp = 5.0
    ki = 0.00005
    kd = 0.01


    as5600_simulation_node = Node(
        package=package_name,
        executable="as5600_simulation",
        parameters=[
            {
                "as5600_publisher_timer": as5600_publisher_timer,
                "counter": counter,
                "min_value": min_value,
                "max_value": max_value,
            }
        ]
    )

    ina219_simulation_node = Node(
        package=package_name,
        executable="ina219_simulation",
        parameters=[
            {
                "ina219_publisher_timer": ina219_publisher_timer,
                "counter": counter,
                "min_value": min_value,
                "max_value": max_value,
            }
        ]
    )

    ads1115_simulation_node = Node(
        package=package_name,
        executable="ads1115_simulation",
        parameters=[
            {
                "ads1115_publisher_timer": ads1115_publisher_timer,
                "counter": counter,
                "min_value": min_value,
                "max_value": max_value,
            }
        ]
    )

    motor_control_node = Node(
        package=package_name,
        executable="motor_control",
        parameters=[
            {
                "target_angle_publisher_timer": target_angle_publisher_timer,
                "pid_exercise_timer": pid_timer,
                "pid_training_timer": pid_timer,
                "pid_free_timer": pid_timer,
                "kp": kp,
                "ki": ki,
                "kd": kd,
                
            }
        ]
    )

    ld.add_action(as5600_simulation_node)
    ld.add_action(ina219_simulation_node)
    ld.add_action(ads1115_simulation_node)
    ld.add_action(motor_control_node)

def generate_launch_description():
    ld = LaunchDescription()
    package_name = "knee_rehab_py_pkg"
    
    init_simulation(ld, package_name)

    return ld
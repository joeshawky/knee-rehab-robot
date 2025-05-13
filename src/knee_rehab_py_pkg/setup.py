from setuptools import find_packages, setup

package_name = 'knee_rehab_py_pkg'
submodules = "knee_rehab_py_pkg/utilities"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['knee_rehab_py_pkg.utilities.knee_rehab_modes', 
                'knee_rehab_py_pkg.utilities.bluetooth', 
                'knee_rehab_py_pkg.utilities.i2c_devices',
                'knee_rehab_py_pkg.utilities.pid_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='joe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bluetooth_manager = knee_rehab_py_pkg.bluetooth_manager:main",
            "as5600_simulation = knee_rehab_py_pkg.as5600_simulation:main",
            "ads1115_simulation = knee_rehab_py_pkg.ads1115_simulation:main",
            "ina219_simulation = knee_rehab_py_pkg.ina219_simulation:main",
            "power_sensing = knee_rehab_py_pkg.power_sensing:main",
            "motor_control = knee_rehab_py_pkg.motor_control:main",
            "i2c_data = knee_rehab_py_pkg.i2c_data:main",
        ],
    },
)

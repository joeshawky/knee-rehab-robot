import smbus
import time

class AS5600:
    """AS5600 magnetic rotary position sensor."""

    # Default I2C Address
    DEFAULT_ADDRESS = 0x36

    # Register addresses
    RAW_ANGLE_REGISTER = 0x0C
    ANGLE_REGISTER = 0x0E
    STATUS_REGISTER = 0x0B
    CONFIG_REGISTER = 0x07
    MAGNITUDE_REGISTER = 0x1B

    # Status bit masks
    STATUS_MD = 0x20  # Magnet detected
    STATUS_MH = 0x08  # Magnet too strong
    STATUS_ML = 0x10  # Magnet too weak

    # Configuration bit masks
    PWM_OUT = 0x0040  # PWM output enabled

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS):
        """Initialize AS5600 sensor."""
        self.bus = i2c_bus
        self.address = address

    def _read_register(self, reg, length=2):
        """Read a 16-bit value (or specified length) from a register."""
        data = self.bus.read_i2c_block_data(self.address, reg, length)
        if length == 2:
            return (data[0] << 8) | data[1]
        elif length == 1:
            return data[0]
        else:
            raise ValueError("Invalid read length specified.")

    def _write_register(self, reg, value):
        """Write an 8-bit or 16-bit value to a register."""
        if value < 256:  # 8-bit write
            self.bus.write_byte_data(self.address, reg, value)
        else:  # 16-bit write
            high_byte = (value >> 8) & 0xFF
            low_byte = value & 0xFF
            self.bus.write_i2c_block_data(self.address, reg, [high_byte, low_byte])

    def read_raw_angle(self):
        """Read raw angle data (0-4095)."""
        raw_angle = self._read_register(self.RAW_ANGLE_REGISTER)
        return raw_angle

    def read_angle(self):
        """Read the angle in degrees (0-360)."""
        raw_angle = self.read_raw_angle()
        return (raw_angle * 360.0) / 4096.0

    def read_status(self):
        """Read the status register and interpret magnet detection flags."""
        status = self._read_register(self.STATUS_REGISTER, 1)
        magnet_detected = bool(status & self.STATUS_MD)
        magnet_too_strong = bool(status & self.STATUS_MH)
        magnet_too_weak = bool(status & self.STATUS_ML)
        return {
            "magnet_detected": magnet_detected,
            "magnet_too_strong": magnet_too_strong,
            "magnet_too_weak": magnet_too_weak,
        }

    def configure_pwm(self, enable_pwm=True):
        """Enable or disable PWM output."""
        config = self._read_register(self.CONFIG_REGISTER)
        if enable_pwm:
            config |= self.PWM_OUT
        else:
            config &= ~self.PWM_OUT
        self._write_register(self.CONFIG_REGISTER, config)

    def read_magnitude(self):
        """Read the magnitude of the magnetic field."""
        magnitude = self._read_register(self.MAGNITUDE_REGISTER)
        return magnitude



class INA219:
    """INA219 current sensor with configuration options."""

    # Default I2C Address
    DEFAULT_ADDRESS = 0x40

    # Register addresses
    CONFIG_REGISTER = 0x00
    SHUNT_VOLTAGE_REGISTER = 0x01
    BUS_VOLTAGE_REGISTER = 0x02
    POWER_REGISTER = 0x03
    CURRENT_REGISTER = 0x04
    CALIBRATION_REGISTER = 0x05

    # Configuration register bit masks
    RESET = 0x8000  # Reset bit

    # Bus voltage range
    BRNG_16V = 0x0000  # 16V range
    BRNG_32V = 0x2000  # 32V range (default)

    # Gain settings (PGA)
    GAIN_1_40MV = 0x0000  # ±40 mV range
    GAIN_2_80MV = 0x0800  # ±80 mV range
    GAIN_4_160MV = 0x1000  # ±160 mV range
    GAIN_8_320MV = 0x1800  # ±320 mV range (default)

    # ADC resolution and averaging
    ADCRES_12BIT_1S = 0x0018  # 12-bit resolution, 1 sample (default)

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS, shunt_resistance=0.1, max_expected_current=3.2):
        """Initialize INA219 with default settings."""
        self.bus = i2c_bus
        self.address = address
        self.shunt_resistance = shunt_resistance
        self.max_expected_current = max_expected_current

        # Default configuration
        self.bus_voltage_range = self.BRNG_32V
        self.gain = self.GAIN_8_320MV
        self.shunt_adc_resolution = self.ADCRES_12BIT_1S
        self.bus_adc_resolution = self.ADCRES_12BIT_1S

        # Calibration value
        self.calibration_value = 0
        self.current_lsb = 0

        # Initialize the sensor
        self.reset()
        self.calibrate()

    def reset(self):
        """Reset the INA219."""
        self._write_register(self.CONFIG_REGISTER, self.RESET)
        time.sleep(0.1)  # Wait for reset

    def calibrate(self):
        """Calibrate the INA219 for the configured shunt resistor and expected current."""
        # Calculate current LSB
        self.current_lsb = self.max_expected_current / 32767  # Current LSB in amperes
        self.calibration_value = int(0.04096 / (self.current_lsb * self.shunt_resistance))

        # Write calibration value
        self._write_register(self.CALIBRATION_REGISTER, self.calibration_value)

    def configure(self):
        """Configure INA219 settings."""
        config = (
            self.bus_voltage_range |
            self.gain |
            self.shunt_adc_resolution |
            self.bus_adc_resolution |
            0x8000
        )
        self._write_register(self.CONFIG_REGISTER, config)

    def read_shunt_voltage(self):
        """Read shunt voltage in millivolts."""
        raw = self._read_register(self.SHUNT_VOLTAGE_REGISTER)
        return raw * 0.01  # Shunt voltage is scaled to 10 µV/LSB

    def read_bus_voltage(self):
        """Read bus voltage in volts."""
        raw = self._read_register(self.BUS_VOLTAGE_REGISTER)
        return (raw >> 3) * 0.004  # Bus voltage is scaled to 4 mV/LSB

    def read_current(self):
        """Read current in amperes."""
        raw = self._read_register(self.CURRENT_REGISTER)
        if raw & 0x8000:  # Handle negative current
            raw -= 1 << 16
        return raw * self.current_lsb

    def _write_register(self, reg, value):
        """Write a 16-bit value to a register."""
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        self.bus.write_i2c_block_data(self.address, reg, [high_byte, low_byte])

    def _read_register(self, reg):
        """Read a 16-bit value from a register."""
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        raw_value = (data[0] << 8) | data[1]
        return raw_value




class ADS1115:
    """ADS1115 ADC with configurable settings for differential or single-ended readings."""

    # Default I2C Address
    DEFAULT_ADDRESS = 0x48

    # Register addresses
    CONVERSION_REGISTER = 0x00
    CONFIG_REGISTER = 0x01

    # Config register bit masks
    OS_SINGLE = 0x8000  # Start a single conversion
    MODE_CONTINUOUS = 0x0000  # Continuous conversion mode
    MODE_SINGLE = 0x0100  # Single-shot mode

    # Input multiplexer (MUX) settings
    MUX_DIFF_0_1 = 0x0000  # Differential P=AIN0, N=AIN1
    MUX_DIFF_2_3 = 0x3000  # Differential P=AIN2, N=AIN3
    MUX_SINGLE_0 = 0x4000  # Single-ended AIN0
    MUX_SINGLE_1 = 0x5000  # Single-ended AIN1

    # Programmable Gain Amplifier (PGA) settings (± voltage ranges)
    PGA_6_144V = 0x0000  # ±6.144V
    PGA_4_096V = 0x0200  # ±4.096V (default)
    PGA_2_048V = 0x0400  # ±2.048V
    PGA_1_024V = 0x0600  # ±1.024V
    PGA_0_512V = 0x0800  # ±0.512V
    PGA_0_256V = 0x0A00  # ±0.256V

    # Data rate settings (samples per second)
    DR_8SPS = 0x0000
    DR_16SPS = 0x0020
    DR_32SPS = 0x0040
    DR_64SPS = 0x0060
    DR_128SPS = 0x0080  # Default
    DR_250SPS = 0x00A0
    DR_475SPS = 0x00C0
    DR_860SPS = 0x00E0

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS, pga=PGA_4_096V, data_rate=DR_128SPS, mode=MODE_SINGLE):
        """Initialize ADS1115 with configurable settings."""
        self.bus = i2c_bus
        self.address = address
        self.pga = pga
        self.data_rate = data_rate
        self.mode = mode

    def _write_register(self, reg, value):
        """Write a 16-bit value to a register."""
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        self.bus.write_i2c_block_data(self.address, reg, [high_byte, low_byte])

    def _read_register(self, reg):
        """Read a 16-bit value from a register."""
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (data[0] << 8) | data[1]

    def read_differential(self, mux=MUX_DIFF_0_1):
        """Read differential voltage."""
        # Configure ADC for differential reading
        config = (
            self.OS_SINGLE   # Start single conversion
            | mux            # MUX setting
            | self.pga       # Programmable Gain Amplifier
            | self.mode      # Conversion mode
            | self.data_rate # Data rate
        )
        self._write_register(self.CONFIG_REGISTER, config)

        # Wait for the conversion to complete (only in single-shot mode)
        if self.mode == self.MODE_SINGLE:
            time.sleep(1 / (8 << ((self.data_rate & 0xE0) >> 5)))  # Calculate wait time based on SPS

        # Read the conversion result
        raw_value = self._read_register(self.CONVERSION_REGISTER)
        if raw_value & 0x8000:  # Negative value handling
            raw_value -= 1 << 16
        return raw_value * (4.096 / 32768.0)  # Convert to voltage


    

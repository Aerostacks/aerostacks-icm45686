"""ICM-45686 register map, option tables and conversion constants."""

STANDARD_GRAVITY = 9.80665

# Value of PWR_MGMT0[3:0] that puts accelerometer and gyroscope in low-noise mode.
LOW_NOISE_MODES = 0x0F

# TDK and the Linux kernel reserve -32768 (0x8000) for "no valid data".
INVALID_VALUE = -32768

ACCEL_RANGES = {32: 0, 16: 1, 8: 2, 4: 3, 2: 4}
GYRO_RANGES = {
    4000: 0,
    2000: 1,
    1000: 2,
    500: 3,
    250: 4,
    125: 5,
    62.5: 6,
    31.25: 7,
    15.625: 8,
}
# Output data rate (Hz) -> ODR field code, for rates usable in low-noise mode.
ODR_CODES = {
    6400: 3,
    3200: 4,
    1600: 5,
    800: 6,
    400: 7,
    200: 8,
    100: 9,
    50: 10,
    25: 11,
    12.5: 12,
}
# UI low-pass filter divisor (cutoff = ODR / divisor) -> bandwidth field code.
FILTER_DIVISORS = {4: 1, 8: 2, 16: 3, 32: 4, 64: 5, 128: 6}

# Default ceiling on the output rate when samples are read by register polling.
# Faster rates need the FIFO, which this driver does not use.
DEFAULT_MAX_RATE_HZ = 200

# Default polling schedule (seconds): wake this long before the next sample is
# due, then poll status at this interval. Tune both for your bus.
DEFAULT_POLL_LEAD_S = 0.0005
DEFAULT_POLL_INTERVAL_S = 0.001

# Direct registers.
DATA_REGISTER = 0x00
PWR_MGMT0 = 0x10
INT1_CONFIG0 = 0x16
STATUS_REGISTER = 0x19
ACCEL_CONFIG0 = 0x1B
GYRO_CONFIG0 = 0x1C
WHO_AM_I = 0x72
IREG_ADDR = 0x7C
IREG_DATA = 0x7E
REG_MISC2 = 0x7F

# Indirect (IREG) registers.
GYRO_SRC_CTRL = 0xA4A6
GYRO_UI_LPF = 0xA4AC
ACCEL_SRC_CTRL = 0xA57B
ACCEL_UI_LPF = 0xA583

DEVICE_ID = 0xE9
READY_MASK = 0x04
SOFT_RESET = 0x02
RESET_DONE = 0x80
TEMP_SENSITIVITY = 128.0

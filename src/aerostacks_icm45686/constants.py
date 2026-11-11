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
# Faster rates need the FIFO (see FifoConfig; its rates are limited only by
# ODR_CODES).
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
# FIFO / APEX / timestamp registers (datasheet DS-000577 rev 1.0 section 17).
FIFO_COUNT_0 = 0x12  # 17.18, FIFO_DATA_CNT[15:8] in big-endian mode
FIFO_COUNT_1 = 0x13  # 17.19, FIFO_DATA_CNT[7:0] in big-endian mode
FIFO_DATA = 0x14  # 17.20, FIFO data port
FIFO_CONFIG0 = 0x1D  # 17.28, FIFO_MODE[7:6], FIFO_DEPTH[5:0]
FIFO_CONFIG2 = 0x20  # 17.31, FIFO_FLUSH[7]
FIFO_CONFIG3 = 0x21  # 17.32, HIRES_EN[3], GYRO_EN[2], ACCEL_EN[1], IF_EN[0]
FIFO_CONFIG4 = 0x22  # 17.33, COMP_EN[2], TMST_FSYNC_EN[1]
TMST_WOM_CONFIG = 0x23  # 17.34, TMST_DELTA_EN[6], TMST_RESOL[5]
ODR_DECIMATE_CONFIG = 0x28  # 17.39, gyro[7:4] / accel[3:0] FIFO decimation
EDMP_APEX_EN0 = 0x29  # 17.40, SMD/R2W/FF/PEDO/TILT/TAP enables
EDMP_APEX_EN1 = 0x2A  # 17.41, EDMP_ENABLE[6], FEATURE3_EN[5], ...
WHO_AM_I = 0x72
IREG_ADDR = 0x7C
IREG_DATA = 0x7E
REG_MISC2 = 0x7F

# Indirect (IREG) registers.
# IPREG_TOP1 registers are addressed as 0xA200 + offset (section 14.4/14.5).
SMC_CONTROL_0 = 0xA258  # 20.47, TMST_EN[0]
SREG_CTRL = 0xA267  # 20.50, SREG_DATA_ENDIAN_SEL[1]
GYRO_SRC_CTRL = 0xA4A6
GYRO_UI_LPF = 0xA4AC
ACCEL_SRC_CTRL = 0xA57B
ACCEL_UI_LPF = 0xA583

DEVICE_ID = 0xE9
READY_MASK = 0x04
SOFT_RESET = 0x02
RESET_DONE = 0x80
TEMP_SENSITIVITY = 128.0

# --- FIFO (datasheet section 6, 17.28, 17.32-17.34, 20.47, 20.50) ---
FIFO_MODE_MASK = 0xC0
FIFO_MODES = {"bypass": 0b00, "stream": 0b01, "stop_on_full": 0b10}
FIFO_DEPTH_MASK = 0x3F
FIFO_DEPTH_CODES = {2048: 0b000111, 8192: 0b011111}  # 8 KB: APEX off only
FIFO_FLUSH = 0x80
FIFO_CONFIG3_HIRES_EN = 0x08
FIFO_CONFIG3_GYRO_EN = 0x04
FIFO_CONFIG3_ACCEL_EN = 0x02
FIFO_CONFIG3_IF_EN = 0x01
FIFO_CONFIG3_MASK = 0x3F  # incl. ES0/ES1 enables, which are forced to 0
FIFO_CONFIG4_MASK = 0x3E  # COMP_NC_FLOW_CFG[5:3], COMP_EN[2], TMST_FSYNC_EN[1]
FIFO_CONFIG4_TMST_FSYNC_EN = 0x02
TMST_WOM_CONFIG_MASK = 0x60  # TMST_DELTA_EN[6], TMST_RESOL[5]
TMST_RESOL_16US = 0x20
SMC_TMST_EN = 0x01
SREG_BIG_ENDIAN = 0x02
APEX_EN0_MASK = 0xF9  # bits 2:1 reserved
APEX_EN1_MASK = 0x67  # EDMP_ENABLE, FEATURE3_EN, POWER_SAVE, INIT, HARD_IRON

# Section 6.1: 20-byte high-resolution packet.
FIFO_PACKET_BYTES = 20
# Section 6.2 header: EXT_HEADER[7]=0, ACCEL_EN[6]=1, GYRO_EN[5]=1, HIRES_EN[4]=1,
# TMST_FIELD_EN[3]=1. Bit 2 (FSYNC_TAG_EN) and bits 1:0 (ODR change flags) vary.
FIFO_HEADER_MASK = 0xF8
FIFO_HEADER_HIRES = 0x78
# Not in the datasheet; TDK's reference driver treats these as "no packet".
FIFO_EMPTY_HEADERS = (0x80, 0xFF)
# Largest I2C_RDWR message the Linux i2c-dev driver accepts.
I2C_MAX_READ_BYTES = 8192

# Section 6: hi-res data is always +/-32 g (16384 LSB/g) and +/-4000 dps
# (2**19 / 4000 = 131.072 LSB/dps; the datasheet rounds this to 131.1).
HIRES_ACCEL_LSB_PER_G = 16384.0
HIRES_GYRO_LSB_PER_DPS = 524288 / 4000
# 20-bit signed gyro; |raw| at or above this is clipped. Accel is 19-bit with
# LSB 0, so its largest positive value is 524286.
HIRES_FULL_SCALE_RAW = 524287
HIRES_ACCEL_FULL_SCALE_RAW = 524286
# Sections 17.13/17.14 TEMP_DATA: degC = raw / 128 + 25 (the 16-bit hi-res field is
# assumed to use the register scaling; the datasheet only spells out the
# 8-bit form, degC = raw / 2 + 25).
FIFO_TEMP_SENSITIVITY = 128.0

# Timestamp: 16 bits, 1 us resolution when TMST_RESOL=0.
TMST_WRAP = 1 << 16
FIFO_MIN_RATE_HZ = 50  # slower rates risk a >= 65.5 ms period (one wrap)

import numpy as np

# 1. EEPROM based codes
ADDR_MODEL_NUMBER = 0
ADDR_MODEL_INFORMATION = 2
ADDR_FIRMWARE_VERSION = 6
ADDR_ID = 7
ADDR_BAUD_RATE = 8
ADDR_RETURN_DELAY_TIME = 9
ADDR_DRIVE_MODE = 10
ADDR_OPERATING_MODE = 11
ADDR_SHADOW_ID = 12
ADDR_PROTOCOL_TYPE = 13
ADDR_HOMING_OFFSET = 20
ADDR_MOVING_THRESHOLD = 24
ADDR_TEMPERATURE_LIMIT = 31
ADDR_MAX_VOLTAGE_LIMIT = 32
ADDR_MIN_VOLTAGE_LIMIT = 34
ADDR_PWM_LIMIT = 36
ADDR_CURRENT_LIMIT = 38
ADDR_VELOCITY_LIMIT = 44
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_SHUTDOWN = 63

# 1. Ram based codes
ADDR_TORQUE = 64
ADDR_LED = 65
ADDR_STATUS_RETURN_LEVEL = 68
ADDR_REGISTERED_INSTRUCTION = 69
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_VELOCITY_I_GAIN = 76
ADDR_VELOCITY_P_GAIN = 78
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_FEEDFORWARD_SECOND_GAIN = 88
ADDR_FEEDFORWARD_FIRST_GAIN = 90
ADDR_BUS_WATCHDOG = 98
ADDR_GOAL_PWM = 100
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_REALTIME_TICK = 120
ADDR_MOVING = 122
ADDR_MOVING_STATUS = 123
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_VELOCITY_TRAJECTORY = 136
ADDR_POSITION_TRAJECTORY = 140
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
ADDR_LEN_GROUP_SET = 4
ADDR_EXTERNAL_PORT = 152

LEN_PRESENT_POSITION = 4
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_CURRENT = 2
LEN_GOAL_POSITION = 4

COMM_SUCCESS = 0
PROTOCOL_VERSION = 2.0

MIN_POSITION = 0
MAX_POSITION = 4095
MIN_RADIAN = -3.14159265
MAX_RADIAN = 3.14159265
CURRENT_UNIT = 2.63


# Macro for Control Table Value
def DXL_MAKEWORD(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)


def DXL_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16


def DXL_LOWORD(l):
    return l & 0xFFFF


def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF


def DXL_LOBYTE(w):
    return w & 0xFF


def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF


def convertRadian2Value(radian):
    value = 0
    zero_position = (MAX_POSITION + MIN_POSITION) / 2

    value = np.where(
        radian > 0,
        (radian * (MAX_POSITION - zero_position) / MAX_RADIAN) + zero_position,
        (radian * (MIN_POSITION - zero_position) / MIN_RADIAN) + zero_position,
    )

    return value.astype(int)


def convertValue2Radian(value):
    radian = 0.0
    zero_position = (MAX_POSITION + MIN_POSITION) / 2

    # Calculate radians using numpy operations
    radian = np.where(
        value > zero_position,
        (value - zero_position) * MAX_RADIAN / (MAX_POSITION - zero_position),
        (value - zero_position) * MIN_RADIAN / (MIN_POSITION - zero_position),
    )

    return radian


def convertVelocity2Value(velocity):
    rpm = 0.229
    RPM2RADPERSEC = 0.104719755

    value = np.where(velocity >= 0, velocity / (rpm * RPM2RADPERSEC), velocity / (rpm * RPM2RADPERSEC) + 0xFFFFFFFF)
    return np.round(value).astype(int)


def convertValue2Velocity(value):
    rpm = 0.229
    RPM2RADPERSEC = 0.104719755

    velocity = np.where(value < int(0xFFFFFFFF / 2), value * rpm * RPM2RADPERSEC, (value - 0xFFFFFFFF) * rpm * RPM2RADPERSEC)
    return velocity


def convertCurrent2Value(current):
    value = current / CURRENT_UNIT
    return value.astype(int)


def convertValue2Current(value):
    # Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102
    # https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#y2cc93fd
    # https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/67a163199cab651fdf4aceb984f3b75ea30847fe/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp#L1465

    current = unsigned_to_signed(value) * CURRENT_UNIT
    return current


def unsigned_to_signed(unsigned_array):
    mask = unsigned_array > 32767
    signed_array = unsigned_array.copy()
    signed_array[mask] -= 65536
    return signed_array


"""
https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/master/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp
typedef struct
{
  float rpm;

  int64_t value_of_MIN_RADIAN_position;
  int64_t value_of_zero_radian_position;
  int64_t value_of_MAX_RADIAN_position;

  float  MIN_RADIAN;
  float  MAX_RADIAN;
} ModelInfo;

static const ModelInfo info_XM = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

static const ModelInfo info_EXTXM = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};
"""


"""
Following converter support only "XM-series".
"""

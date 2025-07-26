import time
from dynamixel_sdk import *
from .dynamixel_utils import *


class DynamixelCore:
    def serial_open(self):
        """
        open the port to the given device of the controller
        """
        try:
            self.port_handler.openPort()
            self.port_handler.setBaudRate(self.baudrate)
        except Exception as error:
            print(error)

    def serial_close(self):
        """
        close the port of the connection to the controller
        """
        self.port_handler.closePort()

    def reboot(self, dxl_id):
        """
        reboot a dynamixel
        :param dxl_id: the id of the dynamixel
        """
        dxl_result, dxl_error = self.packet_handler.reboot(self.port_handler, dxl_id)
        self._validate(dxl_result, dxl_error)

    def factory_reset(self, dxl_id):
        """
        reset a dynamixel to their default values
        :param dxl_id: the id of the dynamixel
        """
        dxl_result, dxl_error = self.packet_handler.factoryReset(
            self.port_handler, dxl_id
        )
        self._validate(dxl_result, dxl_error)

    def get_torque(self, dxl_id):
        """
        get the current torque state of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the torque state
        """
        dxl_torque = self._get_small_register(dxl_id, ADDR_TORQUE)
        return dxl_torque

    def get_led(self, dxl_id):
        """
        get the current led state of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the led state
        """
        dxl_led = self._get_small_register(dxl_id, ADDR_LED)
        return dxl_led

    def get_id(self, dxl_id):
        """
        get the the current id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the id
        """
        dxl_identifier = self._get_small_register(dxl_id, ADDR_ID)
        return dxl_identifier

    def get_shadow_id(self, dxl_id):
        """
        get the current shadow id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the shadow id
        """
        dxl_shadow_id = self._get_small_register(dxl_id, ADDR_SHADOW_ID)
        return dxl_shadow_id

    def get_drive_mode(self, dxl_id):
        """
        get the current drive mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the drive mode
        """
        dxl_drive_mode = self._get_small_register(dxl_id, ADDR_DRIVE_MODE)
        return dxl_drive_mode

    def get_operating_mode(self, dxl_id):
        """
        get the current operating mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the operating mode
        """
        dxl_operating_mode = self._get_small_register(dxl_id, ADDR_OPERATING_MODE)
        return dxl_operating_mode

    def get_model_number(self, dxl_id):
        """
        get the model number of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the model number of the dynamixel
        """
        dxl_model_number = self._get_medium_register(dxl_id, ADDR_MODEL_NUMBER)
        return dxl_model_number

    def get_firmware_version(self, dxl_id):
        """
        get the firmware version of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the firmware version
        """
        dxl_firmware_version = self._get_small_register(dxl_id, ADDR_FIRMWARE_VERSION)
        return dxl_firmware_version

    def get_protocol_type(self, dxl_id):
        """
        get the protocol type of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the protocol type
        """
        dxl_protocol_type = self._get_small_register(dxl_id, ADDR_PROTOCOL_TYPE)
        return dxl_protocol_type

    def get_velocity_kp_gain(self, dxl_id):
        """
        get the velocity p gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the velocity p gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_VELOCITY_P_GAIN)
        return dxl_gain

    def get_velocity_ki_gain(self, dxl_id):
        """
        get the velocity i gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the velocity i gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_VELOCITY_I_GAIN)
        return dxl_gain

    def get_position_kp_gain(self, dxl_id):
        """
        get the position p gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position p gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_POSITION_P_GAIN)
        return dxl_gain

    def get_position_ki_gain(self, dxl_id):
        """
        get the position i gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position i gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_POSITION_I_GAIN)
        return dxl_gain

    def get_position_kd_gain(self, dxl_id):
        """
        get the position d gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the position d gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_POSITION_D_GAIN)
        return dxl_gain

    def get_feedforward_first_gain(self, dxl_id):
        """
        get the feedforward first gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the feedforward first gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_FEEDFORWARD_FIRST_GAIN)
        return dxl_gain

    def get_feedforward_second_gain(self, dxl_id):
        """
        get the feedforward second gain of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the feedforward second gain
        """
        dxl_gain = self._get_medium_register(dxl_id, ADDR_FEEDFORWARD_SECOND_GAIN)
        return dxl_gain

    def get_present_temperature(self, dxl_id):
        """
        get the present temperature of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present temperature
        """
        dxl_temperature = self._get_small_register(dxl_id, ADDR_PRESENT_TEMPERATURE)
        return dxl_temperature

    def get_present_pwm(self, dxl_id):
        """
        get the present pwm of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present pwm
        """
        dxl_pwm = self._get_large_register(dxl_id, ADDR_PRESENT_PWM)
        return dxl_pwm

    def get_present_current(self, dxl_id):
        """
        get the present current of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present current
        """
        dxl_load = self._get_medium_register(dxl_id, ADDR_PRESENT_CURRENT)
        return dxl_load

    def get_present_input_voltage(self, dxl_id):
        """
        get the present input voltage of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present input voltage
        """
        dxl_input_voltage = self._get_medium_register(
            dxl_id, ADDR_PRESENT_INPUT_VOLTAGE
        )
        return dxl_input_voltage

    def get_present_velocity(self, dxl_id):
        """
        get the present velocity of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present velocity
        """
        dxl_velocity = self._get_large_register(dxl_id, ADDR_PRESENT_VELOCITY)
        if dxl_velocity == 0xFFFFFFFF:
            dxl_velocity = 0

        return dxl_velocity

    def get_present_position(self, dxl_id):
        """
        get the present position of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the present position
        """
        dxl_position = self._get_large_register(dxl_id, ADDR_PRESENT_POSITION)
        if dxl_position > int(0xFFFFFFFF / 2):
            dxl_position -= 0xFFFFFFFF

        return dxl_position

    # def get_present_ext_position(self, dxl_id):
    #     """
    #     get the present position of the dynamixel
    #     :param dxl_id: the id of the dynamixel
    #     :return: the present position
    #     """
    #     dxl_position = self._get_large_register(dxl_id, ADDR_PRESENT_POSITION)
    #     if dxl_position > int(0xFFFFFFFF / 2):
    #         dxl_position -= 0xFFFFFFFF
    #     return dxl_position

    def get_realtime_tick(self, dxl_id):
        """
        get the realtime tick of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the realtime tick
        """
        dxl_realtime_tick = self._get_medium_register(dxl_id, ADDR_REALTIME_TICK)
        return dxl_realtime_tick

    def get_external_port(self, dxl_id, port_no=1):
        """
        get the current led state of the dynamixel
        :param dxl_id: the id of the dynamixel
        :return: the led state
        """
        _ADDR_TARGET = ADDR_EXTERNAL_PORT - (port_no - 1) * 2
        state = self._get_medium_register(dxl_id, _ADDR_TARGET)
        return state

    def set_torque(self, dxl_id, torque_state):
        """
        set the torque of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param torque_state: the torque as int
        """
        self._set_small_register(dxl_id, ADDR_TORQUE, torque_state)

    def set_delay_time(self, dxl_id, value):
        """
        set the delay time of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param value: the delay time as int
        """
        self._set_small_register(dxl_id, ADDR_RETURN_DELAY_TIME, value)

    def set_led(self, dxl_id, led_state):
        """
        set the led of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param led_state: led state as bool
        """
        self._set_small_register(dxl_id, ADDR_LED, led_state)

    def set_id(self, dxl_id, identifier):
        """
        set the dynamixel id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param identifier: the id as int from 0-255
        """
        self._set_small_register(dxl_id, ADDR_ID, identifier)

    def set_shadow_id(self, dxl_id, shadow_id):
        """
        set the shadow id of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param shadow_id: the shadow id as int from 0-255
        """
        self._set_small_register(dxl_id, ADDR_SHADOW_ID, shadow_id)

    def set_operating_mode(self, dxl_id, control_mode):
        """
        set the operating mode of the dynamixel
        list of control modes:
          Current Control Mode            = 0
          Velocity Control Mode           = 1
          Position Control Mode           = 3
          Extended Pos. Control Mode      = 4
          Current-Base Pos. Control Mode  = 5
          PWM Control Mode                = 16
        :param dxl_id: the id of the dynamixel
        :param control_mode: the operating mode as int
        """
        self._set_small_register(dxl_id, ADDR_OPERATING_MODE, control_mode)

    def set_drive_mode(self, dxl_id, drive_mode):
        """
        list of drive modes:
        1. Drive Mode Normal    = 0
        2. Drive Mode Reversed  = 1
        set the drive mode of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param drive_mode: the drive mode as int
        """
        self._set_small_register(dxl_id, ADDR_DRIVE_MODE, drive_mode)

    def set_goal_velocity(self, dxl_id, velocity):
        """
        set the goal velocity of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param velocity: the velocity as int, depends on model
        """
        self._set_large_register(dxl_id, ADDR_GOAL_VELOCITY, velocity)

    def set_goal_position(self, dxl_id, position):
        """
        set the goal position of the dynamixel
        :param dxl_id: the id of the dynamixel
        :param position: the goal position as int from 0-4096, depends on model
        """
        self._set_large_register(dxl_id, ADDR_GOAL_POSITION, position)

    def set_velocity_kp_gain(self, dxl_id, gain):
        """
        set the velocity p gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_VELOCITY_P_GAIN, gain)

    def set_velocity_ki_gain(self, dxl_id, gain):
        """
        set the velocity i gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_VELOCITY_I_GAIN, gain)

    def set_position_kp_gain(self, dxl_id, gain):
        """
        set the position p gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_POSITION_P_GAIN, gain)

    def set_position_ki_gain(self, dxl_id, gain):
        """
        set the position i gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_POSITION_I_GAIN, gain)

    def set_position_kd_gain(self, dxl_id, gain):
        """
        set the position d gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_POSITION_D_GAIN, gain)

    def set_feedforward_first_gain(self, dxl_id, gain):
        """
        set the feedforward first gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_FEEDFORWARD_FIRST_GAIN, gain)

    def set_feedforward_second_gain(self, dxl_id, gain):
        """
        set the feedforward first gain
        :param dxl_id: the id of the dynamixel
        :param gain: the gain as int
        """
        self._set_medium_register(dxl_id, ADDR_FEEDFORWARD_SECOND_GAIN, gain)

    def set_current_limit(self, dxl_id, value):
        """
        set the current limit value
        :param dxl_id: the id of the dynamixel
        :param value: the value as int
        """
        self._set_medium_register(dxl_id, ADDR_CURRENT_LIMIT, value)

    def set_position_limit(self, dxl_id, min_value, max_value):
        """
        set the max and min position
        :param dxl_id: the id of the dynamixel
        :param min_value: the min value as int
        :param max_value: the max value as int
        """
        self._set_large_register(dxl_id, ADDR_MIN_POSITION_LIMIT, min_value)
        self._set_large_register(dxl_id, ADDR_MAX_POSITION_LIMIT, max_value)

    def set_homing_offset(self, dxl_id, value):
        """
        set the homing offset
        :param dxl_id: the id of the dynamixel
        :param value: the value as int
        """
        self._set_large_register(dxl_id, ADDR_HOMING_OFFSET, value)

    # MARK: PRIVATE REGISTER FUNCTIONS
    def _set_small_register(self, dxl_id, address, value):
        """
        set a small register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._validate(dxl_result, dxl_error)
        time.sleep(0.01)

    def _set_medium_register(self, dxl_id, address, value):
        """
        set a medium register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._validate(dxl_result, dxl_error)
        time.sleep(0.01)

    def _set_large_register(self, dxl_id, address, value):
        """
        set a large register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :param value: the value to set
        """
        dxl_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._validate(dxl_result, dxl_error)
        time.sleep(0.01)

    def _get_small_register(self, dxl_id, address):
        """
        get a small register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, address
        )
        self._validate(dxl_result, dxl_error)
        return dxl_data

    def _get_medium_register(self, dxl_id, address):
        """
        get a medium register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, address
        )
        self._validate(dxl_result, dxl_error)
        return dxl_data

    def _get_large_register(self, dxl_id, address):
        """
        get a large register entry
        :param dxl_id: the id of the dynamixel
        :param address: dynamixel register address
        :return: the register entry
        """
        dxl_data, dxl_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, address
        )
        self._validate(dxl_result, dxl_error)
        return dxl_data

    def _validate(self, dxl_result, dxl_error):
        """
        validate register set result's
        :param dxl_result: result value
        :param dxl_error: error value
        """
        if dxl_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_result))
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))

    def _sync_value(self, value):
        """
        sync value for group register set
        :param value: the id of the dynamixel
        :return: the synced data set
        """
        data = [
            DXL_LOBYTE(DXL_LOWORD(value)),
            DXL_HIBYTE(DXL_LOWORD(value)),
            DXL_LOBYTE(DXL_HIWORD(value)),
            DXL_HIBYTE(DXL_HIWORD(value)),
        ]
        return data

    # def set_group_goal_position(self, positions):
    #     """
    #     set the goal position of a dynamixel group
    #     :param positions: the position of the dynamixel
    #     """
    #     # param_goal_position = [DXL_LOBYTE(DXL_LOWORD(2048)), DXL_HIBYTE(DXL_LOWORD(2048)),
    #     #                       DXL_LOBYTE(DXL_HIWORD(2048)), DXL_HIBYTE(DXL_HIWORD(2048))]

    #     for dxl_id, position in zip(self.dxl_ids, positions):
    #         # param_goal_position = self._sync_value(position)

    #         # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    #         dxl_result = self.group_sync_write.addParam(dxl_id, param_goal_position)
    #         if dxl_result != True:
    #             print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
    #             quit()

    #     # Syncwrite goal position
    #     dxl_result = self.group_sync_write.txPacket()
    #     self._validate(dxl_result, 0)
    #     self.group_sync_write.clearParam()

    # def get_group_present_position(self):
    #     """
    #     get the present position of the dynamixel
    #     :param dxl_ids: the id of the dynamixel
    #     :return: the present positions
    #     """
    #     dxl_result = self.group_sync_read.txRxPacket()
    #     self._validate(dxl_result, 0)
    #     position = np.array([self.group_sync_read.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for id in self.dxl_ids])
    #     velocity = np.array([self.group_sync_read.getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) for id in self.dxl_ids])
    #     current = np.array([self.group_sync_read.getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT) for id in self.dxl_ids])

    #     return position, velocity, current

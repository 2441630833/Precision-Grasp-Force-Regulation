from enum import Enum
import can
import logging

class MotorController:
    class FUNC_CODES:
        POS_CONTROL = 0xFD  # Position control
        ORIGIN_SET_O = 0x93 # Set single-turn homing zero point
        ORIGIN_TRIGGER_RETURN = 0x9A  # Homing control
        INTERRUPT_ORIGIN_RETURN = 0x9C # Forcefully interrupt and exit the homing operation
        ORIGIN_MODIFY_PARAMS = 0x4C # Modify homing parameters
        RESET_CUR_POS_TO_ZERO = 0x0A # Reset the current position to zero
        RESET_CLOG_PRO = 0x0E # Reset the blockage protection
        EN_CONTROL = 0xF3 # Enable signal control
        VEL_CONTROL = 0xF6 # Velocity mode
        STOP_NOW = 0xFE # Immediate stop (applicable to all control modes)
        SYNCHRONOUS_MOTION = 0xFF # Multi-motor synchronous motion
        S_CONF_W = 0x48 # Modify drive configuration parameters

        # System parameter function codes, can be used as func_code parameter for read_sys_param()
        S_VER = 0x1F  # Read firmware version and corresponding hardware version
        S_RL = 0x20  # Read phase resistance and phase inductance
        S_PID = 0x21  # Read PID parameters
        S_VBUS = 0x24  # Read bus voltage
        S_CPHA = 0x27  # Read phase current
        S_ENCL = 0x31  # Read encoder value after linearization calibration
        S_TPOS = 0x33  # Read motor target position angle
        S_VEL = 0x35  # Read motor real-time speed
        S_CPOS = 0x36  # Read motor real-time position angle
        S_PERR = 0x37  # Read motor position error angle
        S_FLAG = 0x3A  # Read enable/arrived/blocked status flags
        S_ORG = 0x3B  # Read homing/homing failure status flags
        S_CONF = 0x42  # Read drive parameters
        S_STATE = 0x43  # Read system status parameters
        

    class O_MODE(Enum):
        SINGLE_TURN_NEAREST_ZERO = 0  # Single-turn nearest zero
        SINGLE_TURN_DIRECTIONAL_ZERO = 1  # Single-turn directional zero
        MULTI_TURN_UNLIMITED_COLLISION_ZERO = 2  # Multi-turn unlimited collision zero
        MULTI_TURN_LIMITED_SWITCH_ZERO = 3  # Multi-turn limited switch zero

    def __init__(self, bus, addr=1):
        self.bus = bus
        self.addr = addr
        self.DECODER = {
            self.FUNC_CODES.S_VEL: self.decode_velocity,
            self.FUNC_CODES.S_CPOS: self.decode_position,
            self.FUNC_CODES.S_TPOS: self.decode_position,
            self.FUNC_CODES.S_PERR: self.decode_position,
            self.FUNC_CODES.S_CPHA: self.decode_phase_current,
            self.FUNC_CODES.S_FLAG: self.decode_status_flags,
            self.FUNC_CODES.S_CONF: self.decode_config_parameters,
        }

    def pos_control(self, vel, clk, dir=1, acc=100, raF=False, snF=False):
        """
        Position Control.
        Position control

        Args:
        vel (int): Speed (RPM), range 0 - 5000RPM
        clk (int): Pulse count, range 0 - (2^32 - 1)
        dir (int): Direction, 0 for CW, other values for CCW
        acc (int): Acceleration, range 0 - 255, note: 0 means direct start
        raF (bool): Phase/Absolute flag, false for relative movement, true for absolute movement
        snF (bool): Multi-motor synchronization flag, false for disabled, true for enabled
        """
        # Create command data
        cmd_data = bytearray([dir])
        cmd_data += vel.to_bytes(2, byteorder="big")
        cmd_data.append(acc)
        cmd_data += clk.to_bytes(4, byteorder="big")
        cmd_data.append(1 if raF else 0)
        cmd_data.append(1 if snF else 0)
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr, self.FUNC_CODES.POS_CONTROL] + list(cmd_data)

        # Send command
        self.can_send(cmd)
    
    def origin_set_o(self, svF):
        """
        Set the zero position for single-turn homing.
        Set the zero position for single-turn homing
        You can rotate the motor to the desired position, then send this command to set the zero position for single-turn homing.

        Args:
        svF (bool): Whether to store the setting (False: do not store, True: store)
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.ORIGIN_SET_O])
        cmd_data.append(0x88)  # Auxiliary code
        cmd_data.append(1 if svF else 0)  # Store flag
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)
    
    def origin_trigger_return(self, o_mode: O_MODE, snF: bool):
        """
        Origin Trigger Return Control.
        Homing control

        Args:
        o_mode (O_MODE): Homing mode
        snF (bool): Multi-motor synchronization flag (False: disabled, True: enabled)
        """
        if not isinstance(o_mode, self.O_MODE):
            raise ValueError("o_mode must be an instance of O_MODE Enum.")

        # Create command data
        cmd_data = bytearray([o_mode.value])
        cmd_data.append(1 if snF else 0)
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr, self.FUNC_CODES.ORIGIN_TRIGGER_RETURN] + list(cmd_data)

        # Send command
        self.can_send(cmd)
    
    def interrupt_origin_return(self):
        """
        Forcefully interrupt and exit the homing operation.
        Forcefully interrupt and exit the homing operation
        """
        # Create command data
        cmd_data = bytearray([0x48])  # Command specific byte
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr, self.FUNC_CODES.INTERRUPT_ORIGIN_RETURN] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def origin_modify_params(self, svF, o_mode, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, potF):
        """
        Modify homing parameters.
        Modify homing parameters

        Args:
        svF (bool): Whether to store the setting
        o_mode (int): Homing mode
        o_dir (int): Homing direction
        o_vel (int): Homing velocity (RPM)
        o_tm (int): Homing timeout (ms)
        sl_vel (int): Slow-collision detection speed (RPM)
        sl_ma (int): Slow-collision detection current (Ma)
        sl_ms (int): Slow-collision detection time (Ms)
        potF (bool): Power-on trigger for homing
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.ORIGIN_MODIFY_PARAMS])
        cmd_data.append(0xAE)  # Auxiliary code
        cmd_data.append(1 if svF else 0)
        cmd_data.append(o_mode)
        cmd_data.append(o_dir)
        cmd_data.extend(o_vel.to_bytes(2, byteorder='big'))
        cmd_data.extend(o_tm.to_bytes(4, byteorder='big'))
        cmd_data.extend(sl_vel.to_bytes(2, byteorder='big'))
        cmd_data.extend(sl_ma.to_bytes(2, byteorder='big'))
        cmd_data.extend(sl_ms.to_bytes(2, byteorder='big'))
        cmd_data.append(1 if potF else 0)
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def reset_cur_pos_to_zero(self):
        """
        Reset the current position to zero.
        Reset the current position to zero
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.RESET_CUR_POS_TO_ZERO])
        cmd_data.append(0x6D)  # Auxiliary code
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)
    
    def reset_clog_pro(self):
        """
        Reset the blockage protection.
        Reset the blockage protection
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.RESET_CLOG_PRO])
        cmd_data.append(0x52)  # Auxiliary code
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)
    
    
    def en_control(self, state, snF=False):
        """
        Enable signal control.
        Enable signal control

        Args:
        state (bool): Enable state (True to enable the motor, False to disable)
        snF (bool): Multi-motor synchronization flag (False: not enabled, True: enabled)
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.EN_CONTROL])
        cmd_data.append(0xAB)  # Auxiliary code
        cmd_data.append(1 if state else 0)  # Enable state
        cmd_data.append(1 if snF else 0)  # Multi-motor synchronization flag
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def vel_control(self, vel, dir=1, acc=100, snF=False):
        """
        Velocity control.
        Velocity mode
        Args:
        vel (int): Velocity (Range: 0 - 5000 RPM)
        dir (int): Direction (0 for CW, other values for CCW)
        acc (int): Acceleration (Range: 0 - 255, note: 0 for direct start)
        snF (bool): Multi-motor synchronization flag (False: not enabled, True: enabled)
        """
        if not 0 <= vel <= 5000:
            raise ValueError("Velocity must be in the range 0 - 5000 RPM.")
        if not 0 <= acc <= 255:
            raise ValueError("Acceleration must be in the range 0 - 255.")

        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.VEL_CONTROL])
        cmd_data.append(dir)
        cmd_data.extend(vel.to_bytes(2, byteorder='big'))
        cmd_data.append(acc)
        cmd_data.append(1 if snF else 0)
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def stop_now(self, snF=False):
        """
        Immediate stop for the motor (applicable to all control modes).
        Immediate stop (applicable to all control modes)

        Args:
        snF (bool): Multi-motor synchronization flag (False: not enabled, True: enabled)
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.STOP_NOW])
        cmd_data.append(0x98)  # Auxiliary code
        cmd_data.append(1 if snF else 0)  # Multi-motor synchronization flag
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [self.addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def synchronous_motion(self, addr=0):
        """
        Initiate multi-motor synchronous motion.
        Multi-motor synchronous motion. Before sending this command, you need to set the
        target position (pos_control) or velocity (vel_control) for each motor that needs to be
        synchronized and set snF=True to put them in synchronization mode first.

        Args:
        addr (int): Address of the motor controller to be synchronized.
        Default is 0, which means broadcast to notify all motors to start synchronous motion.
        """
        # Create command data
        cmd_data = bytearray([self.FUNC_CODES.SYNCHRONOUS_MOTION])
        cmd_data.append(0x66)  # Auxiliary code
        cmd_data.append(0x6B)  # Check byte

        # Prepare the command
        cmd = [addr] + list(cmd_data)

        # Send command
        self.can_send(cmd)

    def set_config_parameters(self, config_params):
        """
        Set the configuration parameters for the motor controller.

        Args:
        config_params (dict): A dictionary containing key-value pairs of configuration parameters.

        The method constructs a command based on the configuration parameters and sends it to the motor controller.
        """
        cmd = [self.addr, self.FUNC_CODES.S_CONF_W, 0xD1, 0x01]

        # Add each parameter to the command
        cmd.append(config_params["motor_type"])
        cmd.append(config_params["pulse_port_control_mode"])
        cmd.append(config_params["communication_port_mode"])
        cmd.append(config_params["en_pin_effective_level"])
        cmd.append(config_params["dir_pin_effective_direction"])
        cmd.append(config_params["subdivision"])
        cmd.append(config_params["subdivision_interpolation"])
        cmd.append(config_params["auto_screen_off"])
        cmd.extend(config_params["open_loop_mode_current"].to_bytes(2, byteorder='big'))
        cmd.extend(config_params["closed_loop_mode_max_current"].to_bytes(2, byteorder='big'))
        cmd.extend(config_params["closed_loop_mode_max_output_voltage"].to_bytes(2, byteorder='big'))
        cmd.append(config_params["uart_baud_rate"])
        cmd.append(config_params["can_communication_rate"])
        cmd.append(config_params["id_address"])
        cmd.append(config_params["communication_check_method"])
        cmd.append(config_params["control_command_response"])
        cmd.append(config_params["blockage_protection"])
        cmd.extend(config_params["blockage_protection_speed_threshold"].to_bytes(2, byteorder='big'))
        cmd.extend(config_params["blockage_protection_current_threshold"].to_bytes(2, byteorder='big'))
        cmd.extend(config_params["blockage_protection_time_threshold"].to_bytes(2, byteorder='big'))
        cmd.extend(config_params["position_arrival_window"].to_bytes(2, byteorder='big'))

        # Add check byte
        cmd.append(0x6B)

        # Send command
        self.can_send(cmd)
        logging.info(f"Sent configuration parameters: {[hex(x) for x in cmd]}")

    def read_sys_param(self, func_code, clear_can_rx=True):
        """
        Read system parameters from the motor controller.
        Read system parameters

        Args:    
        func_code (int): Function code to read.
        clear_can_rx (bool): Whether to clear the CAN receive buffer before sending the command.
        """
        cmd = [self.addr, func_code]

        # Special handling for S_CONF and S_STATE
        if func_code in [self.FUNC_CODES.S_CONF, self.FUNC_CODES.S_STATE]:
            cmd.append(0x6C if func_code == self.FUNC_CODES.S_CONF else 0x7A)

        cmd.append(0x6B)  # Check byte
        if clear_can_rx:
            self.clear_can_rx()
        # Send command
        self.can_send(cmd)
        return self.receive_sys_param(func_code)

    # Decode methods
    def decode_velocity(self, data):
        if len(data) != 5:
            raise ValueError("Invalid data length for velocity.")
        vel = (data[2] << 8) | data[3]
        if data[1]:
            vel = -vel
        return vel

    def decode_position(self, data):
        if len(data) != 7:
            raise ValueError("Invalid data length for position.")
        pos = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
        position = (float(pos) * 360.0) / 65536.0
        if data[1]:
            position = -position
        return position

    def decode_phase_current(self, data):
        """
        Decode the phase current from CAN data.

        Data Format: 0x27 + Phase Current (2 bytes) + Check Byte
        """
        if len(data) != 4:
            raise ValueError("Invalid data length for phase current.")

        # Extract and calculate phase current
        phase_current = (data[1] << 8) | data[2]  # Combine two bytes
        return phase_current  # Returns current in milliamperes (mA)

    def decode_status_flags(self, data):
        """
        Decode the motor status flags from CAN data.

        Data Format: 0x3A + Status Flags Byte + Check Byte
        Returns a tuple of four booleans representing the motor status flags.
        """
        if len(data) != 3:
            raise ValueError("Invalid data length for status flags.")

        flags_byte = data[1]
        enabled = bool(flags_byte & 0x01)  # Motor enable status flag
        position_arrived = bool(flags_byte & 0x02)  # Motor arrived flag
        blocked = bool(flags_byte & 0x04)  # Motor blocked flag
        blocked_protection = bool(flags_byte & 0x08)  # Motor blocked protection flag

        return enabled, position_arrived, blocked, blocked_protection
    def decode_config_parameters(self, data):
        """
        Decode the configuration parameters from CAN data.

        Data Format: 0x42 + Configuration Parameters + Check Byte
        Returns a dictionary of configuration parameters.
        """
        if len(data) != 32 or data[0] != self.FUNC_CODES.S_CONF:
            data_hex_list = [hex(byte) for byte in data]
            raise ValueError(f"Invalid data length or function code for configuration parameters. received: {data_hex_list}, length: {len(data_hex_list)}")

        config_params = {
            "motor_type": data[3],
            "pulse_port_control_mode": data[4],
            "communication_port_mode": data[5],
            "en_pin_effective_level": data[6],
            "dir_pin_effective_direction": data[7],
            "subdivision": data[8],
            "subdivision_interpolation": data[9],
            "auto_screen_off": data[10],
            "open_loop_mode_current": int.from_bytes(data[11:13], byteorder='big'),
            "closed_loop_mode_max_current": int.from_bytes(data[13:15], byteorder='big'),
            "closed_loop_mode_max_output_voltage": int.from_bytes(data[15:17], byteorder='big'),
            "uart_baud_rate": data[17],
            "can_communication_rate": data[18],
            "id_address": data[19],
            "communication_check_method": data[20],
            "control_command_response": data[21],
            "blockage_protection": data[22],
            "blockage_protection_speed_threshold": int.from_bytes(data[23:25], byteorder='big'),
            "blockage_protection_current_threshold": int.from_bytes(data[25:27], byteorder='big'),
            "blockage_protection_time_threshold": int.from_bytes(data[27:29], byteorder='big'),
            "position_arrival_window": int.from_bytes(data[29:31], byteorder='big'),
        }

        return config_params
    def receive_sys_param(self, expected_func_code):
        """
        Receive system parameters from the motor controller.

        Args:
        expected_func_code (int): The expected function code of the received message.
        """
        if expected_func_code == self.FUNC_CODES.S_CONF:
            return self.receive_multiple_sys_params(expected_func_code)
        else:
            message = self.bus.recv(timeout=0.1)  # timeout in seconds
            if not message:
                raise TimeoutError("No response received from the CAN bus.")

            if message.data[0] != expected_func_code:
                raise ValueError("Received unexpected function code.")

            if expected_func_code in self.DECODER:
                return self.DECODER[expected_func_code](message.data)
            else:
                raise ValueError("No decode method for function code.")

    def receive_multiple_sys_params(self, expected_func_code):
        """
        Receive multiple system parameter messages from the motor controller.
        Specifically handles the S_CONF function code.
        """
        messages = []
        while True:
            message = self.bus.recv(timeout=0.1)  # timeout in seconds
            if not message or (message.data[0] != expected_func_code and len(messages) > 0):
                break  # Exit loop if no more messages or different function code

            messages.append(message)

        # Concatenate data, skip function code in subsequent messages
        data = bytearray()
        for i, msg in enumerate(messages):
            if i == 0:  # Include function code for the first message
                data.extend(msg.data)
            else:  # Skip function code for subsequent messages
                data.extend(msg.data[1:])

        return self.decode_config_parameters(data)

    def clear_can_rx(self, timeout=1):
        """
        Clear all existing messages in the receive buffer.

        Parameters:
        timeout: Time to wait for the clearing operation (in milliseconds).
        """
        timeout_seconds = timeout / 1000.0  # Convert to seconds
        while True:
            received_msg = self.bus.recv(timeout=timeout_seconds)
            if received_msg is None:
                break
            logging.info(f"Cleared message with ID {hex(received_msg.arbitration_id)}")

    def can_send(self, cmd):
        addr, func_code, *data = cmd
        messages = []
        pack_num = 0
        i = 0
        while i < len(data):
            # Function code + data packet
            message_data = bytearray([func_code]) + bytearray(
                data[i : min(i + 7, len(data))]
            )
            i += 7
            # High 8 bits represent motor ID address, low 8 bits represent which data packet (0 represents the first packet)
            ext_id = (addr << 8) | pack_num
            messages.append(
                can.Message(
                    arbitration_id=ext_id, data=message_data, is_extended_id=True
                )
            )
            pack_num += 1

        for message in messages:
            logging.info(f"Sending message: {message}")
            self.bus.send(message)

from enum import Enum
import can
import logging

class MotorController:
    class FUNC_CODES:
        POS_CONTROL = 0xFD  # 位置控制
        ORIGIN_SET_O = 0x93 # 设置单圈回零的零点位置
        ORIGIN_TRIGGER_RETURN = 0x9A  # 回零控制
        INTERRUPT_ORIGIN_RETURN = 0x9C # 强制中断并退出回零操作
        ORIGIN_MODIFY_PARAMS = 0x4C # 修改回零参数
        RESET_CUR_POS_TO_ZERO = 0x0A # 将当前位置清零
        RESET_CLOG_PRO = 0x0E # 解除堵转保护
        EN_CONTROL = 0xF3 # 使能信号控制
        VEL_CONTROL = 0xF6 # 速度模式
        STOP_NOW = 0xFE # 立即停止（所有控制模式都通用）
        SYNCHRONOUS_MOTION = 0xFF # 多机同步运动
        S_CONF_W = 0x48 # 修改驱动配置参数

        # 系统参数功能码，可作为read_sys_param()的func_code参数
        S_VER = 0x1F  # 读取固件版本和对应的硬件版本
        S_RL = 0x20  # 读取相电阻和相电感
        S_PID = 0x21  # 读取PID参数
        S_VBUS = 0x24  # 读取总线电压
        S_CPHA = 0x27  # 读取相电流
        S_ENCL = 0x31  # 读取经过线性化校准后的编码器值
        S_TPOS = 0x33  # 读取电机目标位置角度
        S_VEL = 0x35  # 读取电机实时转速
        S_CPOS = 0x36  # 读取电机实时位置角度
        S_PERR = 0x37  # 读取电机位置误差角度
        S_FLAG = 0x3A  # 读取使能/到位/堵转状态标志位
        S_ORG = 0x3B  # 读取正在回零/回零失败状态标志位
        S_CONF = 0x42  # 读取驱动参数
        S_STATE = 0x43  # 读取系统状态参数
        

    class O_MODE(Enum):
        SINGLE_TURN_NEAREST_ZERO = 0  # 单圈就近回零
        SINGLE_TURN_DIRECTIONAL_ZERO = 1  # 单圈方向回零
        MULTI_TURN_UNLIMITED_COLLISION_ZERO = 2  # 多圈无限位碰撞回零
        MULTI_TURN_LIMITED_SWITCH_ZERO = 3  # 多圈有限位开关回零

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
        位置控制

        Args:
        vel (int): 速度(RPM)   ，范围0 - 5000RPM
        clk (int): 脉冲数      ，范围0- (2^32 - 1)个
        dir (int): 方向        ，0为CW，其余值为CCW
        acc (int): 加速度      ，范围0 - 255，注意：0是直接启动
        raF (bool): 相位/绝对标志，false为相对运动，true为绝对值运动
        snF (bool): 多机同步标志 ，false为不启用，true为启用
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
        设置单圈回零的零点位置
        可以让电机转到想要的位置， 然后发送该命令设置单圈回零的零点位置。

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
        回零控制

        Args:
        o_mode (O_MODE): 回零模式
        snF (bool): 多机同步运动标志 (False: 不启用, True: 启用)
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
        强制中断并退出回零操作
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
        修改回零参数

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
        将当前位置清零
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
        解除堵转保护
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
        使能信号控制

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
        速度模式
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
        立即停止（所有控制模式都通用）

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
        多机同步运动，发送此指令前，需要设置各需要同步运行电机的
        目标位置（pos_control）或速度（vel_control）并设置snF=True使其先
        进入同步模式。

        Args:
        addr (int): Address of the motor controller to be synchronized.
        默认0，表示广播通知所有电机开始同步运动。
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
        读取系统参数

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
        enabled = bool(flags_byte & 0x01)  # 电机使能状态标志位
        position_arrived = bool(flags_byte & 0x02)  # 电机到位标志位
        blocked = bool(flags_byte & 0x04)  # 电机堵转标志位
        blocked_protection = bool(flags_byte & 0x08)  # 电机堵转保护标志

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
        清除接收缓冲区中的所有现有消息。

        参数:
        timeout: 等待清除操作的时间（单位：毫秒）。
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
            # 功能码+数据包
            message_data = bytearray([func_code]) + bytearray(
                data[i : min(i + 7, len(data))]
            )
            i += 7
            # 高8位表示电机ID地址，低8位表示第几包数据(0表示第一包)。
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

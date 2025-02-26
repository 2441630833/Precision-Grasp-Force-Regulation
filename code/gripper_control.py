import logging
from stepper_motor import MotorController
from time import sleep


class GripperControl:
    def __init__(self, bus, addr=1, max_clk_open=32000):
        self.motor = MotorController(bus, addr=addr)
        self.s_conf = self.motor.read_sys_param(self.motor.FUNC_CODES.S_CONF)
        self.max_clk_open = max_clk_open
        self.CLK_DIV_DEGREE = 8.88893  # clk/degree, proportion constant
        self.OPEN = 0
        self.CLOSE = 1
        self.DEFAULT_ACC = 253
        self.DEFAULT_VEL = 500

    def set_max_output_voltage(self, voltage_mv, refresh_conf=False):
        if refresh_conf or self.s_conf is None:
            self.s_conf = self.motor.read_sys_param(self.motor.FUNC_CODES.S_CONF)
            logging.info(
                f"Current closed-loop mode maximum output voltage: {self.s_conf.get('closed_loop_mode_max_output_voltage')}mV"
            )

        # Modify the maximum output voltage in closed-loop mode, unit mV, maximum can be 5000mV
        self.s_conf["closed_loop_mode_max_output_voltage"] = voltage_mv
        self.motor.set_config_parameters(self.s_conf)

    def init_position(
        self,
        auto_voltage=1000,
        auto_pos_clk=40000,
        min_stop_pos_err=10,
        min_stop_vel=0.1,
    ):
        """Automatically initialize gripper open/close position

        Args:
            auto_voltage (int, optional): Maximum output voltage used during auto initialization. Defaults to 1000.
            auto_pos_clk (int, optional): Maximum displacement used during auto initialization, must be greater than maximum travel. Defaults to 40000.
            min_stop_pos_err (int, optional): Minimum position error to detect when fully closed. Defaults to 10.
            min_stop_vel (float, optional): Maximum current velocity to detect when fully closed. Defaults to 0.1.
        """
        self.set_max_output_voltage(auto_voltage)
        self.motor.reset_cur_pos_to_zero()
        sleep(0.1)
        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=auto_pos_clk,
            dir=self.CLOSE,
            acc=self.DEFAULT_ACC,
            raF=False,
            snF=False,
        )
        for i in range(50):
            sleep(0.2)
            target_degree = self.get_target_position_degree()
            current_degree = self.get_current_position_degree()
            current_vel = self.get_current_velocity()
            if (
                abs(current_vel) < min_stop_vel
                and abs(target_degree - current_degree) > min_stop_pos_err
            ):
                logging.info(
                    f"Actual angle: {current_degree:.2f}, Target-Actual angle difference: {target_degree-current_degree:.2f}, Current velocity: {current_vel:.2f}"
                )
                # Reset current position to zero
                self.motor.reset_cur_pos_to_zero()
                break

        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=self.max_clk_open,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # Absolute movement
            snF=False,
        )

    def get_current_velocity(self):
        return self.motor.read_sys_param(self.motor.FUNC_CODES.S_VEL)

    def get_current_position_degree(self):
        return self.motor.read_sys_param(self.motor.FUNC_CODES.S_CPOS)

    def get_target_position_degree(self):
        return self.motor.read_sys_param(self.motor.FUNC_CODES.S_TPOS)

    def clear_clog(
        self,
        min_stop_pos_err=10,
        min_stop_vel=0.1,
    ):
        target_degree = self.get_target_position_degree()
        current_degree = self.get_current_position_degree()
        current_vel = self.get_current_velocity()
        if (
            abs(current_vel) < min_stop_vel
            and abs(target_degree - current_degree) > min_stop_pos_err
        ):
            new_target_clk = current_degree * self.CLK_DIV_DEGREE
            logging.info(f"clear clog position to clk: {new_target_clk:.2f}")
            self.motor.pos_control(
                vel=self.DEFAULT_VEL,
                clk=int(new_target_clk),
                dir=self.OPEN,
                acc=self.DEFAULT_ACC,
                raF=True, # Absolute movement
                snF=False,
            )
    def close_gripper(self):
        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=0,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # Absolute movement
            snF=False,
        )

    def open_gripper(self):
        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=self.max_clk_open,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # Absolute movement
            snF=False,
        )

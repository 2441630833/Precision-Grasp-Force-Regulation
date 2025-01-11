import logging
from stepper_motor import MotorController
from time import sleep


class GripperControl:
    def __init__(self, bus, addr=1, max_clk_open=32000):
        self.motor = MotorController(bus, addr=addr)
        self.s_conf = self.motor.read_sys_param(self.motor.FUNC_CODES.S_CONF)
        self.max_clk_open = max_clk_open
        self.CLK_DIV_DEGREE = 8.88893  # clk/角度, 比例常数
        self.OPEN = 0
        self.CLOSE = 1
        self.DEFAULT_ACC = 253
        self.DEFAULT_VEL = 500

    def set_max_output_voltage(self, voltage_mv, refresh_conf=False):
        if refresh_conf or self.s_conf is None:
            self.s_conf = self.motor.read_sys_param(self.motor.FUNC_CODES.S_CONF)
            logging.info(
                f"当前闭环模式最大输出电压：{self.s_conf.get('closed_loop_mode_max_output_voltage')}mV"
            )

        # 修改闭环模式最大输出电压，单位mV，最大可以是 5000mV
        self.s_conf["closed_loop_mode_max_output_voltage"] = voltage_mv
        self.motor.set_config_parameters(self.s_conf)

    def init_position(
        self,
        auto_voltage=1000,
        auto_pos_clk=40000,
        min_stop_pos_err=10,
        min_stop_vel=0.1,
    ):
        """自动初始化爪开合位置

        Args:
            auto_voltage (int, optional): 自动初始化时使用的最大输出电压. Defaults to 1000.
            auto_pos_clk (int, optional): 自动初始化时使用的最大位移，需大于最大行程. Defaults to 40000.
            min_stop_pos_err (int, optional): 检测合到头的最小位置误差. Defaults to 10.
            min_stop_vel (float, optional): 加测合到头的最大当前速度. Defaults to 0.1.
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
                    f"实际角度：{current_degree:.2f}, 目标-实际角度差：{target_degree-current_degree:.2f}, 当前速度: {current_vel:.2f}"
                )
                # 将当前位置清零
                self.motor.reset_cur_pos_to_zero()
                break

        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=self.max_clk_open,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # 绝对值运动
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
                raF=True, # 绝对值运动
                snF=False,
            )
    def close_gripper(self):
        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=0,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # 绝对值运动
            snF=False,
        )

    def open_gripper(self):
        self.motor.pos_control(
            vel=self.DEFAULT_VEL,
            clk=self.max_clk_open,
            dir=self.OPEN,
            acc=self.DEFAULT_ACC,
            raF=True, # 绝对值运动
            snF=False,
        )

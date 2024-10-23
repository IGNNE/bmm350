"""BMM350 ROS driver main module"""

import sys
import time

from dataclasses import dataclass
from typing import List, Tuple, Optional

from smbus2 import SMBus, i2c_msg

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

# necessary register values taken straight from the Bosch code
BMM350_ADDR = 0x14
# regs
BMM350_REG_CMD = 0x7E
BMM350_REG_CHIP_ID = 0x00
BMM350_REG_OTP_CMD = 0x50
BMM350_REG_PMU_CMD_AGGR_SET = 0x04
BMM350_REG_PMU_CMD = 0x06
BMM350_REG_PMU_CMD_AXIS_EN = 0x05
BMM350_REG_MAG_X_XLSB = 0x31
BMM350_REG_OTP_STATUS = 0x55
BMM350_REG_OTP_DATA_MSB = 0x52
BMM350_REG_OTP_DATA_LSB = 0x53
# reg values
BMM350_CMD_SOFTRESET = 0xB6
BMM350_OTP_CMD_PWR_OFF_OTP = 0x80
BMM350_ODR_25_HZ = 0x06
BMM350_ODR_12_5_HZ = 0x07
BMM350_AVG_8 = 0x03
BMM350_CMD_NORMAL_MODE = 0x01
BMM350_CMD_UPD_ODR_AVG = 0x02
BMM350_ENABLE_XYZ = 0x1 | 0x2 | 0x4
BMM350_OTP_CMD_DIR_READ = 0x20
BMM350_OTP_WORD_ADDR_MSK = 0x1F
BMM350_ODR_25_HZ_AVG_8 = BMM350_ODR_25_HZ | (BMM350_AVG_8 << 4)
# other consts
BMM350_MAG_OFFSET_X = 0x0E
BMM350_MAG_OFFSET_Y = 0x0F
BMM350_MAG_OFFSET_Z = 0x10
BMM350_TEMP_OFF_SENS = 0x0D
BMM350_MAG_SENS_X = 0x10
BMM350_MAG_SENS_Y = 0x11
BMM350_MAG_SENS_Z = 0x12
BMM350_SENS_CORR_Y = 0.01
BMM350_TCS_CORR_Z = 0.0001
BMM350_MAG_TCO_X = 0x12
BMM350_MAG_TCO_Y = 0x13
BMM350_MAG_TCO_Z = 0x14
BMM350_MAG_TCS_X = 0x12
BMM350_MAG_TCS_Y = 0x13
BMM350_MAG_TCS_Z = 0x13
BMM350_MAG_DUT_T_0 = 0x18
BMM350_CROSS_X_Y = 0x15
BMM350_CROSS_Y_X = 0x15
BMM350_CROSS_Z_X = 0x16
BMM350_CROSS_Z_Y = 0x16
BMM350_PMU_CMD_BR = 0x7
BMM350_PMU_CMD_FGR = 0x05
BMM350_CONVERSION_COEFFS = [
    0.007069978699505961,
    0.007069978699505961,
    0.007174964082129806,
    0.0009812818524089293,
]


@dataclass
class MagComp:
    """A bunch of sensor parameters in one handy dataclass/'struct'"""

    # pylint: disable=too-many-instance-attributes
    dut_offset_coef_x: float = 0.0
    dut_offset_coef_y: float = 0.0
    dut_offset_coef_z: float = 0.0
    dut_offset_coef_t: float = 0.0
    dut_sensit_coef_x: float = 0.0
    dut_sensit_coef_y: float = 0.0
    dut_sensit_coef_z: float = 0.0
    dut_sensit_coef_t: float = 0.0
    dut_tco_x: float = 0.0
    dut_tco_y: float = 0.0
    dut_tco_z: float = 0.0
    dut_tcs_x: float = 0.0
    dut_tcs_y: float = 0.0
    dut_tcs_z: float = 0.0
    dut_t0: float = 0.0
    cross_axis_x_y: float = 0.0
    cross_axis_y_x: float = 0.0
    cross_axis_z_x: float = 0.0
    cross_axis_z_y: float = 0.0


class BMM350(Node):
    """A somewhat hacky BMM350 magnetometer driver node"""

    def __init__(self, **kwargs):
        super().__init__(
            "bmm350", automatically_declare_parameters_from_overrides=True, **kwargs
        )
        # set up topics
        self.publisher = self.create_publisher(MagneticField, "/output/mag", 1)
        # init sensor
        self.otp_data = []
        bus_number = (
            self.get_parameter("bus_number").get_parameter_value().integer_value
        )
        self.bus = SMBus(bus_number)
        # reset
        self.bmm350_write_byte(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)
        time.sleep(0.1)
        chip_id = self.bmm350_read_byte(BMM350_REG_CHIP_ID)
        if chip_id != 0x33:
            self.get_logger().error(
                f"Error: cannot read chip id, expected 0x33, got {chip_id}"
            )
            self.bus.close()
            sys.exit(1)
        # dump calib data
        self.mag_comp = self.otp_dump_after_boot()
        # end otp access
        self.bmm350_write_byte(BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP)
        time.sleep(0.1)
        # magnetic reset on every boot
        self.bmm350_magnetic_reset_and_wait()
        # set data rate and averaging: 8x avg, 25 hz
        self.bmm350_write_byte(BMM350_REG_PMU_CMD_AGGR_SET, BMM350_ODR_25_HZ_AVG_8)
        self.bmm350_write_byte(BMM350_REG_PMU_CMD, BMM350_CMD_UPD_ODR_AVG)
        time.sleep(0.1)
        # enable axes
        self.bmm350_write_byte(BMM350_REG_PMU_CMD_AXIS_EN, BMM350_ENABLE_XYZ)
        # switch from standby to normal mode
        self.bmm350_write_byte(BMM350_REG_PMU_CMD, BMM350_CMD_NORMAL_MODE)
        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        """Function to read and publish the data, to be called periodically"""
        values = self.bmm350_read_temp_compensated()
        mf = MagneticField(header=Header(stamp=self.get_clock().now().to_msg()))
        mf.magnetic_field.x = values[0]
        mf.magnetic_field.y = values[1]
        mf.magnetic_field.z = values[2]
        self.publisher.publish(mf)

    def bmm350_write_byte(self, register: int, value: int):
        """wrapper around smbus api"""
        self.bus.write_byte_data(BMM350_ADDR, register, value)

    def bmm350_read_byte(self, register: int) -> int:
        """bmm350_read_bytes with length 1"""
        return self.bmm350_read_bytes(register, 1)[0]

    def bmm350_read_bytes(self, register: int, length: int) -> List[int]:
        """there is some weird stuff with dummy bytes going on, we need to discard the first two"""

        # this would be limited to 32B:
        # return bus.read_i2c_block_data(address, register, 3*len)[2::3]

        write = i2c_msg.write(BMM350_ADDR, [register])
        read = i2c_msg.read(BMM350_ADDR, length + 2)
        self.bus.i2c_rdwr(write, read)
        return list(read)[2::]

    def fix_sign(self, inp: int, bits: int) -> int:
        """convert from 'uint' to signed"""
        bits -= 1
        return inp - 2 * pow(2, bits) if inp >= pow(2, bits) else inp

    def read_otp_word(self, addr: int) -> int:
        """query chip otp data at addr"""
        self.bmm350_write_byte(
            BMM350_REG_OTP_CMD,
            BMM350_OTP_CMD_DIR_READ | (addr & BMM350_OTP_WORD_ADDR_MSK),
        )
        # this is only called once
        # sowhy poll the status flag when you can just sleep a bit longer
        time.sleep(0.1)
        # read actual data
        lsb = self.bmm350_read_byte(BMM350_REG_OTP_DATA_LSB)
        msb = self.bmm350_read_byte(BMM350_REG_OTP_DATA_MSB)
        return (msb << 8) | lsb

    def otp_dump_after_boot(self) -> MagComp:
        """a mixup of otp_dump_after_boot and update_mag_off_sens from Bosch code"""

        # I know, I know
        # pylint: disable=too-many-locals

        self.otp_data = []
        for i in range(32):
            self.otp_data.append(self.read_otp_word(i))

        # no idea what this does, but the C code does it, so I'm keeping it here for reference
        # var_id = (otp_data[30] & 0x7f00) >> 9

        # copied verbatim from C, idk what happens here either
        self.mag_comp = MagComp()
        off_x_lsb_msb = self.otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF
        off_y_lsb_msb = ((self.otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) + (
            self.otp_data[BMM350_MAG_OFFSET_Y] & 0xFF
        )
        off_z_lsb_msb = (self.otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) + (
            self.otp_data[BMM350_MAG_OFFSET_Z] & 0xFF
        )
        t_off = self.otp_data[BMM350_TEMP_OFF_SENS] & 0xFF
        self.mag_comp.dut_offset_coef_x = self.fix_sign(off_x_lsb_msb, 12)
        self.mag_comp.dut_offset_coef_y = self.fix_sign(off_y_lsb_msb, 12)
        self.mag_comp.dut_offset_coef_z = self.fix_sign(off_z_lsb_msb, 12)
        self.mag_comp.dut_offset_coef_t = self.fix_sign(t_off, 8) / 5.0
        sens_x = (self.otp_data[BMM350_MAG_SENS_X] & 0xFF00) >> 8
        sens_y = self.otp_data[BMM350_MAG_SENS_Y] & 0xFF
        sens_z = (self.otp_data[BMM350_MAG_SENS_Z] & 0xFF00) >> 8
        t_sens = (self.otp_data[BMM350_TEMP_OFF_SENS] & 0xFF00) >> 8
        self.mag_comp.dut_sensit_coef_x = self.fix_sign(sens_x, 8) / 256.0
        self.mag_comp.dut_sensit_coef_y = (
            self.fix_sign(sens_y, 8) / 256.0
        ) + BMM350_SENS_CORR_Y
        self.mag_comp.dut_sensit_coef_z = self.fix_sign(sens_z, 8) / 256.0
        self.mag_comp.dut_sensit_coef_t = self.fix_sign(t_sens, 8) / 512.0
        tco_x = self.otp_data[BMM350_MAG_TCO_X] & 0xFF
        tco_y = self.otp_data[BMM350_MAG_TCO_Y] & 0xFF
        tco_z = self.otp_data[BMM350_MAG_TCO_Z] & 0xFF
        self.mag_comp.dut_tco_x = self.fix_sign(tco_x, 8) / 32.0
        self.mag_comp.dut_tco_y = self.fix_sign(tco_y, 8) / 32.0
        self.mag_comp.dut_tco_z = self.fix_sign(tco_z, 8) / 32.0
        tcs_x = (self.otp_data[BMM350_MAG_TCS_X] & 0xFF00) >> 8
        tcs_y = (self.otp_data[BMM350_MAG_TCS_Y] & 0xFF00) >> 8
        tcs_z = (self.otp_data[BMM350_MAG_TCS_Z] & 0xFF00) >> 8
        self.mag_comp.dut_tcs_x = self.fix_sign(tcs_x, 8) / 16384.0
        self.mag_comp.dut_tcs_y = self.fix_sign(tcs_y, 8) / 16384.0
        self.mag_comp.dut_tcs_z = (
            self.fix_sign(tcs_z, 8) / 16384.0
        ) - BMM350_TCS_CORR_Z
        self.mag_comp.dut_t0 = (
            self.fix_sign(self.otp_data[BMM350_MAG_DUT_T_0], 16) / 512.0
        ) + 23.0
        cross_x_y = self.otp_data[BMM350_CROSS_X_Y] & 0xFF
        cross_y_x = (self.otp_data[BMM350_CROSS_Y_X] & 0xFF00) >> 8
        cross_z_x = self.otp_data[BMM350_CROSS_Z_X] & 0xFF
        cross_z_y = (self.otp_data[BMM350_CROSS_Z_Y] & 0xFF00) >> 8
        self.mag_comp.cross_axis_x_y = self.fix_sign(cross_x_y, 8) / 800.0
        self.mag_comp.cross_axis_y_x = self.fix_sign(cross_y_x, 8) / 800.0
        self.mag_comp.cross_axis_z_x = self.fix_sign(cross_z_x, 8) / 800.0
        self.mag_comp.cross_axis_z_y = self.fix_sign(cross_z_y, 8) / 800.0
        # print(mag_comp)
        return self.mag_comp

    def bmm350_magnetic_reset_and_wait(self):
        """perform the magnetic reset to compensate for exposure to strong fields"""
        # skipping some checks here, because I am lazy and this code is long enough
        # for reference, this is supposed to run in suspend mode
        self.bmm350_write_byte(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)
        time.sleep(0.1)
        self.bmm350_write_byte(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)
        time.sleep(0.1)

    def bmm350_read_temp_compensated(self) -> Tuple[float]:
        """the big function where the magic happens (most of it)

        returns: Tuple of compensated values: (x, y, z, temp)
        """
        # read all mag and temp registers in one go
        mag_data = self.bmm350_read_bytes(BMM350_REG_MAG_X_XLSB, 12)
        # build values from the registers
        raw_mag_x = mag_data[0] + (mag_data[1] << 8) + (mag_data[2] << 16)
        raw_mag_y = mag_data[3] + (mag_data[4] << 8) + (mag_data[5] << 16)
        raw_mag_z = mag_data[6] + (mag_data[7] << 8) + (mag_data[8] << 16)
        raw_temp = mag_data[9] + (mag_data[10] << 8) + (mag_data[11] << 16)
        # correction for gain, offset etc.
        coefficients = [
            0.007069978699505961,
            0.007069978699505961,
            0.007174964082129806,
            0.0009812818524089293,
        ]
        mag_x = self.fix_sign(raw_mag_x, 24) * coefficients[0]
        mag_y = self.fix_sign(raw_mag_y, 24) * coefficients[1]
        mag_z = self.fix_sign(raw_mag_z, 24) * coefficients[2]
        temp = self.fix_sign(raw_temp, 24) * coefficients[3]
        # additional temp offset
        if temp > 0.0:
            temp = temp - 25.49
        elif temp < 0.0:
            temp = temp + 25.49
        # print(f"x {mag_x}\t y {mag_y}\t z {mag_z}\t t {temp}")
        # apply compensation - straight from the C source
        temp = (
            1 + self.mag_comp.dut_sensit_coef_t
        ) * temp + self.mag_comp.dut_offset_coef_t
        # x comp
        mag_x *= 1 + self.mag_comp.dut_sensit_coef_x
        mag_x += self.mag_comp.dut_offset_coef_x
        mag_x += self.mag_comp.dut_tco_x * (temp - self.mag_comp.dut_t0)
        mag_x /= 1 + self.mag_comp.dut_tcs_x * (temp - self.mag_comp.dut_t0)
        # y comp
        mag_y *= 1 + self.mag_comp.dut_sensit_coef_y
        mag_y += self.mag_comp.dut_offset_coef_y
        mag_y += self.mag_comp.dut_tco_y * (temp - self.mag_comp.dut_t0)
        mag_y /= 1 + self.mag_comp.dut_tcs_y * (temp - self.mag_comp.dut_t0)
        # z comp
        mag_z *= 1 + self.mag_comp.dut_sensit_coef_z
        mag_z += self.mag_comp.dut_offset_coef_z
        mag_z += self.mag_comp.dut_tco_z * (temp - self.mag_comp.dut_t0)
        mag_z /= 1 + self.mag_comp.dut_tcs_z * (temp - self.mag_comp.dut_t0)
        # cross comp
        cr_ax_comp_x = (mag_x - self.mag_comp.cross_axis_x_y * mag_y) / (
            1 - self.mag_comp.cross_axis_y_x * self.mag_comp.cross_axis_x_y
        )
        cr_ax_comp_y = (mag_y - self.mag_comp.cross_axis_y_x * mag_x) / (
            1 - self.mag_comp.cross_axis_y_x * self.mag_comp.cross_axis_x_y
        )
        cr_ax_comp_z = mag_z + (
            mag_x
            * (
                self.mag_comp.cross_axis_y_x * self.mag_comp.cross_axis_z_y
                - self.mag_comp.cross_axis_z_x
            )
            - mag_y
            * (
                self.mag_comp.cross_axis_z_y
                - self.mag_comp.cross_axis_x_y * self.mag_comp.cross_axis_z_x
            )
        ) / (1 - self.mag_comp.cross_axis_y_x * self.mag_comp.cross_axis_x_y)
        return (cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z, temp)


def main(args: Optional[List[str]] = None) -> None:
    """Initialize the node and spin till shutdown is requested."""

    rclpy.init(args=args)
    bmm350 = BMM350()
    bmm350.run()

    try:
        rclpy.spin(bmm350)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()

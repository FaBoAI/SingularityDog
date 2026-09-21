"""Hardware-free behavioral tests for the ICM-20948 diagnostic driver."""
import ctypes
import math
import struct
import unittest
from unittest import mock

from singularitydog_hw import imu


class FakeBus:
    """Four real register banks, read-to-clear readiness, injectable bus faults."""

    def __init__(self, power=0x41):
        self.banks = [bytearray(128) for _ in range(4)]
        self.bank = 3
        initial = {
            imu.WHO_AM_I: 0xEA, imu.PWR_MGMT_1: power,
            imu.PWR_MGMT_2: 0x00, imu.LP_CONFIG: 0x70,
            imu.GYRO_SMPLRT_DIV: 27, imu.GYRO_CONFIG_1: 0x03,
            imu.GYRO_CONFIG_2: 0x02, imu.ACCEL_SMPLRT_DIV_1: 0x01,
            imu.ACCEL_SMPLRT_DIV_2: 25, imu.ACCEL_CONFIG: 0x07,
            imu.ACCEL_CONFIG_2: 0x01,
        }
        for (bank, register), value in initial.items():
            self.banks[bank][register] = value
        self.writes = []
        self.reads = []
        self.fail_write = None
        self.ignore_write = None
        self.fail_burst = False
        self.ready_after_burst = False
        self.short_burst = False

    def read(self, address, register, length):
        assert address == 0x68
        self.reads.append((self.bank, register, length))
        if register == imu.REG_BANK_SEL:
            return bytes([self.bank << 4])
        if self.bank == 0 and register == 0x2D and length == 14:
            if self.fail_burst:
                self.fail_burst = False
                raise OSError("injected burst failure")
            result = bytes(self.banks[0][register:register + length])
            if self.ready_after_burst:
                self.banks[0][0x1A] = 1
                self.ready_after_burst = False
            return result[:8] if self.short_burst else result
        result = bytes(self.banks[self.bank][register:register + length])
        if self.bank == 0 and register == 0x1A:
            self.banks[0][0x1A] = 0
        return result

    def write(self, address, register, data):
        assert address == 0x68
        if register == imu.REG_BANK_SEL:
            self.bank = data[0] >> 4
            return
        key = (self.bank, register)
        self.writes.append((key, bytes(data)))
        if self.bank == 2 and self.banks[0][6] & 0x20:
            raise OSError("bank2 write blocked by LP_EN")
        if self.ignore_write == key:
            self.ignore_write = None
            return
        self.banks[self.bank][register:register + len(data)] = data
        if self.fail_write == key:
            self.fail_write = None
            raise OSError("injected error after write reached chip")

    def sample(self, values):
        self.banks[0][0x2D:0x39] = struct.pack(">hhhhhh", *values)
        self.banks[0][0x1A] = 1


class IMUTests(unittest.TestCase):
    def make_driver(self, bus=None):
        bus = bus or FakeBus()
        ticks = iter(range(1000, 100000, 100))
        driver = imu.ICM20948(bus, sleep=lambda _: None,
                              monotonic_ns=lambda: next(ticks),
                              wall_time_ns=lambda: 123456789)
        return bus, driver

    def assert_registers_restored(self, bus, original):
        for key, value in original.items():
            if key == "REG_BANK_SEL":
                self.assertEqual(bus.bank << 4, value)
            else:
                bank, register = key.split(":")
                self.assertEqual(bus.banks[int(bank[4:])][int(register, 16)], value, key)

    def test_readback_ranges_banks_units_and_signed_big_endian(self):
        bus, driver = self.make_driver()
        configuration = driver.start()
        self.assertEqual(configuration["who_am_i"], 0xEA)
        self.assertEqual(configuration["accel_range_g"], 2)
        self.assertEqual(configuration["gyro_range_dps"], 250)
        self.assertEqual(configuration["registers"]["bank2:0x01"], 0x21)
        self.assertEqual(configuration["registers"]["bank0:0x06"], 1)
        bus.sample((16384, -16384, -32768, 131, -262, 32767))
        sample = driver.read_sample()
        self.assertEqual(sample["raw_accel"], [16384, -16384, -32768])
        self.assertEqual(sample["raw_gyro"], [131, -262, 32767])
        self.assertEqual(sample["accel_m_s2"], [9.80665, -9.80665, -19.6133])
        self.assertAlmostEqual(sample["gyro_rad_s"][0], math.pi / 180)
        self.assertAlmostEqual(sample["gyro_rad_s"][1], -math.pi / 90)
        self.assertEqual(sample["frame"], "sensor")
        self.assertFalse(configuration["orientation_applied"])
        self.assertIn((0, 0x2D, 14), bus.reads)
        self.assertEqual(sample["temperature_c"], 21.0)
        self.assertEqual(sample["monotonic_ns"], 1050)
        self.assertEqual(sample["wall_time_ns"], 123456789)
        driver.close()

    def test_temperature_signed_and_disabled(self):
        bus, driver = self.make_driver()
        driver.start()
        bus.sample((0,0,16384,0,0,0))
        bus.banks[0][0x39:0x3B] = struct.pack(">h", -3339)
        self.assertAlmostEqual(driver.read_sample()["temperature_c"], 11.0, places=2)
        driver.close()
        bus, driver = self.make_driver(FakeBus(power=0x49))
        driver.start()
        bus.sample((0,0,16384,0,0,0))
        self.assertIsNone(driver.read_sample()["temperature_c"])
        driver.close()

    def test_trim_audit_preserves_registers_and_returns_to_bank_zero(self):
        bus, driver = self.make_driver()
        driver.start()
        bus.banks[1][0x14:0x16] = bytes.fromhex("fffd")
        bus.banks[2][3:5] = bytes.fromhex("8000")
        bus.banks[1][0x28] = 0xFE
        before = [bytes(b) for b in bus.banks]
        writes = list(bus.writes)
        result = driver.diagnostic_registers()
        self.assertEqual(result["accel_factory_offset"]["x"]["signed15_code"], -2)
        self.assertEqual(result["accel_factory_offset"]["x"]["reserved_low_bit"], 1)
        self.assertEqual(result["gyro_user_offset_signed16"]["x"], -32768)
        self.assertEqual(result["timebase_pll_signed8"], -2)
        self.assertEqual([bytes(b) for b in bus.banks], before)
        self.assertEqual(bus.writes, writes)
        self.assertEqual(bus.bank, 0)
        driver.close()

    def test_ready_is_consumed_but_fresh_identical_values_are_not_deduplicated(self):
        bus, driver = self.make_driver()
        bus.sample((1, 2, 3, 4, 5, 6))  # Must be discarded during setup.
        driver.start()
        self.assertIsNone(driver.read_sample())
        bus.sample((1, 2, 3, 4, 5, 6))
        self.assertEqual(driver.read_sample()["sequence"], 1)
        self.assertIsNone(driver.read_sample())
        bus.sample((1, 2, 3, 4, 5, 6))
        self.assertEqual(driver.read_sample()["sequence"], 2)
        driver.close()

    def test_late_ready_does_not_double_count_same_register_image(self):
        bus, driver = self.make_driver()
        driver.start()
        bus.sample((1, 2, 3, 4, 5, 6))
        bus.ready_after_burst = True
        sample = driver.read_sample()
        self.assertTrue(sample["data_ready_during_read"])
        self.assertIsNone(driver.read_sample())
        driver.close()

    def test_restore_exact_original_state_including_sleep_lp_and_bank(self):
        bus, driver = self.make_driver(FakeBus(power=0x61))
        with driver:
            original = driver.original_registers
            self.assertEqual(bus.banks[0][6], 1)
            self.assertEqual(bus.banks[0][5], 0x40)
            self.assertEqual(bus.banks[0][7], 0)
        self.assert_registers_restored(bus, original)
        self.assertEqual(driver.restore_status, "restored")
        self.assertFalse(any(k == imu.PWR_MGMT_1 and d[0] & 0x80 for k, d in bus.writes))
        self.assertFalse(any(k[0] == 3 or k == imu.USER_CTRL for k, _ in bus.writes))

    def test_context_body_exception_restores_sensor(self):
        bus, driver = self.make_driver()
        with self.assertRaisesRegex(ValueError, "caller"):
            with driver:
                original = driver.original_registers
                raise ValueError("caller failure")
        self.assert_registers_restored(bus, original)

    def test_write_failure_after_reaching_device_is_restored(self):
        bus, driver = self.make_driver()
        bus.fail_write = imu.ACCEL_CONFIG
        with self.assertRaisesRegex(OSError, "after write"):
            driver.start()
        self.assert_registers_restored(bus, driver.original_registers)
        self.assertEqual(driver.restore_status, "restored")

    def test_ignored_configuration_write_fails_readback_and_restores(self):
        bus, driver = self.make_driver()
        bus.ignore_write = imu.GYRO_CONFIG_1
        with self.assertRaisesRegex(imu.IMUError, "read"):
            driver.start()
        self.assert_registers_restored(bus, driver.original_registers)

    def test_burst_failure_automatically_restores_and_ends_acquisition(self):
        bus, driver = self.make_driver()
        driver.start()
        bus.sample((1, 2, 3, 4, 5, 6))
        bus.fail_burst = True
        with self.assertRaisesRegex(OSError, "burst failure"):
            driver.read_sample()
        self.assert_registers_restored(bus, driver.original_registers)
        with self.assertRaisesRegex(imu.IMUError, "start"):
            driver.read_sample()

    def test_short_read_is_rejected_and_restored(self):
        bus, driver = self.make_driver()
        driver.start()
        bus.sample((1, 2, 3, 4, 5, 6))
        bus.short_burst = True
        with self.assertRaisesRegex(OSError, "Short"):
            driver.read_sample()
        self.assert_registers_restored(bus, driver.original_registers)

    def test_restore_attempts_remaining_registers_after_one_failure_and_can_retry(self):
        bus, driver = self.make_driver(FakeBus(power=0x61))
        driver.start()
        bus.fail_write = imu.GYRO_CONFIG_1
        with self.assertRaises(imu.RestoreError):
            driver.restore()
        self.assertEqual(driver.restore_status, "failed")
        self.assertEqual(bus.banks[0][6], 0x61)
        self.assertEqual(bus.banks[2][0x14], 7)
        driver.restore()  # Must clear LP_EN again before retrying bank2 writes.
        self.assert_registers_restored(bus, driver.original_registers)
        self.assertEqual(driver.restore_status, "restored")

    def test_wrong_identity_only_changes_bank_and_restores_it(self):
        bus, driver = self.make_driver()
        bus.banks[0][0] = 0x00
        with self.assertRaisesRegex(imu.IMUError, "WHO_AM_I"):
            driver.start()
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.bank, 3)

    def test_failed_final_bank_restore_blocks_restart_until_retried(self):
        bus, driver = self.make_driver()
        driver.start()
        original_write = bus.write
        fail_once = [True]

        def fail_final_bank(address, register, data):
            if register == 0x7F and data == b"\x30" and fail_once[0]:
                fail_once[0] = False
                raise OSError("final bank restore failed")
            original_write(address, register, data)

        bus.write = fail_final_bank
        with self.assertRaises(imu.RestoreError):
            driver.restore()
        self.assertEqual(bus.bank, 0)
        with self.assertRaisesRegex(imu.IMUError, "Unrestored"):
            driver.start()
        self.assertEqual(driver.original_registers["REG_BANK_SEL"], 0x30)
        driver.restore()
        self.assertEqual(bus.bank, 3)
        driver.start()
        driver.close()
        self.assertEqual(bus.bank, 3)

    def test_active_aux_master_is_rejected_without_reconfiguring_it(self):
        bus, driver = self.make_driver()
        bus.banks[0][3] = 0x20
        with self.assertRaisesRegex(imu.IMUError, "aux-master"):
            driver.start()
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.bank, 3)

    def test_self_test_is_rejected_and_wake_is_restored(self):
        bus, driver = self.make_driver()
        bus.banks[2][2] |= 0x20
        with self.assertRaisesRegex(imu.IMUError, "Self-test"):
            driver.start()
        self.assert_registers_restored(bus, driver.original_registers)

    def test_bad_bank_readback_fails_before_wrong_bank_access(self):
        bus, driver = self.make_driver()
        original_write = bus.write
        ignored = [False]

        def bad_bank(address, register, data):
            if register == 0x7F and data == b"\x20" and not ignored[0]:
                ignored[0] = True
                return
            original_write(address, register, data)

        bus.write = bad_bank
        with self.assertRaisesRegex(imu.IMUError, "bank selection"):
            driver.start()
        self.assert_registers_restored(bus, driver.original_registers)
        self.assertFalse(any(k[0] == 2 for k, _ in bus.writes))


class LinuxI2CTests(unittest.TestCase):
    def test_combined_read_uses_two_messages_with_repeated_start_and_raw_address(self):
        captured = []

        def ioctl(fd, command, argument, mutate):
            self.assertEqual(fd, 42)
            self.assertEqual(command, 0x0707)
            request = imu._I2CTransfer.from_buffer_copy(argument)
            self.assertEqual(request.nmsgs, 2)
            first, second = request.msgs[0], request.msgs[1]
            captured.append((first.addr, first.flags, first.len, first.buf[0],
                             second.addr, second.flags, second.len))
            second.buf[0], second.buf[1] = 0xAB, 0xCD
            return 2

        with mock.patch.object(imu.sys, "platform", "linux"), \
                mock.patch.object(imu.os, "open", return_value=42), \
                mock.patch.object(imu.os, "close") as close, \
                mock.patch.object(imu.fcntl, "ioctl", side_effect=ioctl):
            bus = imu.LinuxI2C()
            self.assertEqual(bus.read(0x68, 0x2D, 2), b"\xAB\xCD")
            bus.close()
            bus.close()
            close.assert_called_once_with(42)
        self.assertEqual(captured, [(0x68, 0, 1, 0x2D, 0x68, 1, 2)])

    def test_incomplete_ioctl_is_an_error(self):
        with mock.patch.object(imu.sys, "platform", "linux"), \
                mock.patch.object(imu.os, "open", return_value=42), \
                mock.patch.object(imu.os, "close"), \
                mock.patch.object(imu.fcntl, "ioctl", return_value=1):
            bus = imu.LinuxI2C()
            with self.assertRaisesRegex(OSError, "Incomplete"):
                bus.read(0x68, 0, 1)
            bus.close()


if __name__ == "__main__":
    unittest.main()

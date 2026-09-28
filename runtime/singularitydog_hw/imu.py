"""ICM-20948 accel/gyro diagnostics, in the unmodified chip register frame.

The bus must be exclusively owned by this process for the session. No reset,
magnetometer access, fusion, bias removal, or mounting rotation is performed.
Register definitions: TDK DS-000189, sections 3, 8, 10 and 14.
"""
from __future__ import annotations

import copy
import ctypes
import errno
import fcntl
import math
import os
import struct
import sys
import time


class IMUError(RuntimeError):
    """Invalid identity, configuration, or session state."""


class RestoreError(IMUError):
    """Restoration was attempted for all touched registers but was incomplete."""

    def __init__(self, failures):
        self.failures = list(failures)
        super().__init__("IMU restoration incomplete: " + "; ".join(self.failures))


class _I2CMessage(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.POINTER(ctypes.c_uint8))]


class _I2CTransfer(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(_I2CMessage)), ("nmsgs", ctypes.c_uint32)]


class LinuxI2C:
    """Small stdlib i2c-dev adapter using combined I2C_RDWR transactions."""

    I2C_RDWR = 0x0707
    I2C_M_RD = 0x0001

    def __init__(self, path="/dev/i2c-7"):
        if not sys.platform.startswith("linux"):
            raise OSError("Linux i2c-dev is required; use an injected bus for tests")
        self.fd = os.open(os.fspath(path), os.O_RDWR | os.O_CLOEXEC)

    def _transfer(self, messages):
        if self.fd is None:
            raise OSError("I2C adapter is closed")
        array = (_I2CMessage * len(messages))(*messages)
        request = _I2CTransfer(array, len(messages))
        # Keep array and its pointed-to buffers alive across the synchronous ioctl.
        completed = fcntl.ioctl(self.fd, self.I2C_RDWR, bytearray(bytes(request)), True)
        if completed != len(messages):
            raise OSError(errno.EIO, "Incomplete I2C_RDWR transfer")

    def read(self, address, register, length):
        if not 1 <= length <= 65535:
            raise ValueError("I2C read length must be 1..65535")
        register_buffer = (ctypes.c_uint8 * 1)(register)
        data = (ctypes.c_uint8 * length)()
        self._transfer([_I2CMessage(address, 0, 1, register_buffer),
                        _I2CMessage(address, self.I2C_M_RD, length, data)])
        return bytes(data)

    def write(self, address, register, data):
        payload = bytes([register]) + bytes(data)
        if len(payload) > 65535:
            raise ValueError("I2C write payload is too large")
        buffer = (ctypes.c_uint8 * len(payload)).from_buffer_copy(payload)
        self._transfer([_I2CMessage(address, 0, len(payload), buffer)])

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)


REG_BANK_SEL = 0x7F
WHO_AM_I = (0, 0x00)
USER_CTRL = (0, 0x03)
LP_CONFIG = (0, 0x05)
PWR_MGMT_1 = (0, 0x06)
PWR_MGMT_2 = (0, 0x07)
INT_ENABLE_1 = (0, 0x11)
INT_STATUS_1 = (0, 0x1A)
GYRO_SMPLRT_DIV = (2, 0x00)
GYRO_CONFIG_1 = (2, 0x01)
GYRO_CONFIG_2 = (2, 0x02)
ACCEL_SMPLRT_DIV_1 = (2, 0x10)
ACCEL_SMPLRT_DIV_2 = (2, 0x11)
ACCEL_CONFIG = (2, 0x14)
ACCEL_CONFIG_2 = (2, 0x15)

_CONFIG_REGISTERS = (GYRO_SMPLRT_DIV, GYRO_CONFIG_1, ACCEL_SMPLRT_DIV_1,
                     ACCEL_SMPLRT_DIV_2, ACCEL_CONFIG)
_RESTORE_ORDER = _CONFIG_REGISTERS + (INT_ENABLE_1, LP_CONFIG, PWR_MGMT_2, PWR_MGMT_1)


def _register_name(key):
    return "bank%d:0x%02X" % key


class ICM20948:
    """Read latest accel/gyro data; context exit restores prior configuration.

    ``bus`` is a Linux device path or an injected object implementing
    read(address, register, length) -> bytes and write(address, register, bytes).
    Construction does not open hardware. ``start`` or context entry does.
    ``read_sample`` returns None when no new data-ready event is pending.
    Explicit ``restore`` is also required after non-context use, normally via
    ``close`` in a finally block. SIGKILL/power loss cannot run restoration.
    """

    def __init__(self, bus="/dev/i2c-7", address=0x68, *, sleep=time.sleep,
                 monotonic_ns=time.monotonic_ns, wall_time_ns=time.time_ns,
                 accel_range_g=2):
        if address not in (0x68, 0x69):
            raise ValueError("ICM-20948 address must be 0x68 or 0x69")
        if accel_range_g not in (2, 4, 8, 16):
            raise ValueError("ICM-20948 accel range must be 2, 4, 8, or 16 g")
        self.address = address
        self._accel_range_g = accel_range_g
        self._bus_argument = bus
        self._owns_bus = isinstance(bus, (str, bytes, os.PathLike))
        self._bus = None if self._owns_bus else bus
        self._sleep = sleep
        self._monotonic_ns = monotonic_ns
        self._wall_time_ns = wall_time_ns
        self._original = {}
        self._original_bank = None
        self._touched = set()
        self._configuration = {}
        self._started = False
        self._closed = False
        self._sequence = 0
        self.restore_status = "not_needed"

    @property
    def original_registers(self):
        result = {_register_name(k): v for k, v in self._original.items()}
        if self._original_bank is not None:
            result["REG_BANK_SEL"] = self._original_bank
        return result

    @property
    def configuration(self):
        return copy.deepcopy(self._configuration)

    def _bus_read(self, register, length=1):
        data = bytes(self._bus.read(self.address, register, length))
        if len(data) != length:
            raise OSError(errno.EIO, "Short ICM-20948 register read")
        return data

    def _select(self, bank):
        # Set and verify explicitly; no bank cache can survive an interrupted write.
        self._bus.write(self.address, REG_BANK_SEL, bytes([bank << 4]))
        if (self._bus_read(REG_BANK_SEL)[0] & 0x30) != bank << 4:
            raise IMUError("Register bank selection did not read back correctly")

    def _read(self, key):
        self._select(key[0])
        return self._bus_read(key[1])[0]

    def _snapshot(self, keys):
        for key in keys:
            if key not in self._original:
                self._original[key] = self._read(key)

    def _write_verified(self, key, value):
        self._select(key[0])
        self._bus.write(self.address, key[1], bytes([value]))
        actual = self._bus_read(key[1])[0]
        if actual != value:
            raise IMUError("%s wrote 0x%02X, read 0x%02X" %
                           (_register_name(key), value, actual))
        return actual

    def _change(self, key, value):
        if key not in self._original:
            raise IMUError("Cannot change an unsnapshotted register")
        if self._read(key) == value:
            return value
        # A failed write may have reached the device: mark before submitting it.
        self._touched.add(key)
        self.restore_status = "pending"
        return self._write_verified(key, value)

    def start(self):
        if self._closed:
            raise IMUError("IMU session is closed")
        if self._started:
            return self.configuration
        if self._touched or self.restore_status == "failed":
            raise IMUError("Unrestored sensor state; call restore before restarting")
        if self._bus is None:
            self._bus = LinuxI2C(self._bus_argument)
        self._original = {}
        self._original_bank = None
        self._configuration = {}
        self.restore_status = "not_needed"
        try:
            self._original_bank = self._bus_read(REG_BANK_SEL)[0] & 0x30
            who = self._read(WHO_AM_I)
            if who != 0xEA:
                raise IMUError("WHO_AM_I expected 0xEA, got 0x%02X" % who)
            self._snapshot((USER_CTRL, PWR_MGMT_1, PWR_MGMT_2, LP_CONFIG))
            if self._original[USER_CTRL] & 0xFE:
                raise IMUError("DMP/FIFO/aux-master or reset is active; exclusive idle IMU required")
            if self._original[PWR_MGMT_1] & 0x80:
                raise IMUError("Device reset is in progress")
            # LP_EN must be zero before bank2 writes (datasheet section14.7).
            awake_power = (self._original[PWR_MGMT_1] & 0x18) | 0x01
            self._change(PWR_MGMT_1, awake_power)
            self._sleep(0.050)
            self._snapshot((INT_ENABLE_1,) + _CONFIG_REGISTERS +
                           (GYRO_CONFIG_2, ACCEL_CONFIG_2))
            if (self._original[GYRO_CONFIG_2] & 0x38 or
                    self._original[ACCEL_CONFIG_2] & 0x1C):
                raise IMUError("Self-test is enabled; idle measurement configuration required")
            # Stop measurement while changing filters/dividers, without a reset.
            self._change(PWR_MGMT_2, self._original[PWR_MGMT_2] | 0x3F)
            self._change(LP_CONFIG, self._original[LP_CONFIG] & ~0x30)
            targets = {
                GYRO_SMPLRT_DIV: 10,
                GYRO_CONFIG_1: (self._original[GYRO_CONFIG_1] & 0xC0) | 0x21,
                ACCEL_SMPLRT_DIV_1: self._original[ACCEL_SMPLRT_DIV_1] & 0xF0,
                ACCEL_SMPLRT_DIV_2: 10,
                ACCEL_CONFIG: ((self._original[ACCEL_CONFIG] & 0xC0) | 0x21 |
                               ((2, 4, 8, 16).index(self._accel_range_g) << 1)),
            }
            for key, value in targets.items():
                self._change(key, value)
            self._change(INT_ENABLE_1, self._original[INT_ENABLE_1] | 1)
            self._change(PWR_MGMT_2, self._original[PWR_MGMT_2] & ~0x3F)
            self._sleep(0.100)  # 35ms typical gyro wake time plus filter settling.
            targets.update({
                PWR_MGMT_1: awake_power,
                PWR_MGMT_2: self._original[PWR_MGMT_2] & ~0x3F,
                LP_CONFIG: self._original[LP_CONFIG] & ~0x30,
                INT_ENABLE_1: self._original[INT_ENABLE_1] | 1,
            })
            actual = {key: self._read(key) for key in targets}
            if actual != targets:
                raise IMUError("Configuration changed during initialization")
            gyro_fs = (actual[GYRO_CONFIG_1] >> 1) & 3
            accel_fs = (actual[ACCEL_CONFIG] >> 1) & 3
            self._accel_scale = 9.80665 / (16384.0, 8192.0, 4096.0, 2048.0)[accel_fs]
            self._gyro_scale = math.pi / 180.0 / (131.0, 65.5, 32.8, 16.4)[gyro_fs]
            self._configuration = {
                "who_am_i": who, "address": self.address, "frame": "sensor",
                "axis_order": ["x", "y", "z"], "orientation_applied": False,
                "magnetometer_initialized": False,
                "accel_range_g": (2, 4, 8, 16)[accel_fs],
                "gyro_range_dps": (250, 500, 1000, 2000)[gyro_fs],
                "accel_m_s2_per_lsb": self._accel_scale,
                "gyro_rad_s_per_lsb": self._gyro_scale,
                "accel_dlpf_hz": 23.9, "gyro_dlpf_hz": 23.9,
                "accel_divider": 10, "gyro_divider": 10,
                "accel_nominal_odr_hz": 1125.0 / 11,
                "gyro_nominal_odr_hz": 1125.0 / 11,
                "odr_note": "Nominal from DS-000189 tables 16/18; gyro section 10.1 rounds base to 1.1 kHz. Measure host sample intervals.",
                "host_poll_hz_requested": 100,
                "registers": {_register_name(k): v for k, v in actual.items()},
            }
            # Drop stale readiness from setup/settling; first returned sample is new.
            self._read(INT_STATUS_1)
            self._started = True
            self._sequence = 0
            return self.configuration
        except BaseException as error:
            self._restore_after_error(error)
            raise

    def read_sample(self):
        """Return latest sensor-frame sample, or None if no data-ready event.

        Timestamps bound the host transaction, not the sensor conversion. This
        is latest-value acquisition: updates can be missed, and identical raw
        values with a fresh ready event remain valid independent samples.
        """
        if not self._started:
            raise IMUError("Call start before read_sample")
        try:
            self._select(0)
            started_ns = self._monotonic_ns()
            status = self._bus_read(INT_STATUS_1[1])[0]
            if not status & 1:
                return None
            raw = struct.unpack(">7h", self._bus_read(0x2D, 14))
            # Clear a new event that raced the burst. Otherwise it could cause
            # the very same register image to be counted again on the next call.
            late_ready = bool(self._bus_read(INT_STATUS_1[1])[0] & 1)
            ended_ns = self._monotonic_ns()
            self._sequence += 1
            return {
                "sequence": self._sequence, "frame": "sensor",
                "monotonic_ns": (started_ns + ended_ns) // 2,
                "wall_time_ns": self._wall_time_ns(),
                "read_started_monotonic_ns": started_ns,
                "read_finished_monotonic_ns": ended_ns,
                "timestamp_source": "host_read_interval_midpoint",
                "wall_time_source": "host_read_finished",
                "data_ready_status": status,
                "data_ready_during_read": late_ready,
                "raw_accel": list(raw[:3]), "raw_gyro": list(raw[3:6]),
                "accel_m_s2": [value * self._accel_scale for value in raw[:3]],
                "gyro_rad_s": [value * self._gyro_scale for value in raw[3:6]],
                "raw_temperature": raw[6],
                "temperature_c": (raw[6] / 333.87 + 21.0
                                  if not self._configuration["registers"]["bank0:0x06"] & 8 else None),
            }
        except BaseException as error:
            self._restore_after_error(error)
            raise

    def diagnostic_registers(self):
        """Read offset/trim bytes without resetting or writing corrections.

        Nonzero factory accel offsets are expected on calibrated parts; this
        snapshot alone cannot establish whether a trim value is correct.
        """
        if not self._started:
            raise IMUError("Call start before diagnostic_registers")
        try:
            raw = {}
            registers = {1: (0x02,0x03,0x04,0x0E,0x0F,0x10,
                             0x14,0x15,0x17,0x18,0x1A,0x1B,0x28),
                         2: (0x03,0x04,0x05,0x06,0x07,0x08)}
            for bank, addresses in registers.items():
                self._select(bank)
                for address in addresses:
                    raw[_register_name((bank,address))] = self._bus_read(address)[0]
            accel = {}
            for axis, address in zip("xyz", (0x14,0x17,0x1A)):
                word = (raw[_register_name((1,address))] << 8) | raw[_register_name((1,address+1))]
                value = word >> 1
                if value & 0x4000:
                    value -= 0x8000
                accel[axis] = {"raw_word_hex": f"{word:04x}", "signed15_code":value,
                               "reserved_low_bit":word & 1}
            gyro = {}
            for axis, address in zip("xyz", (0x03,0x05,0x07)):
                word = (raw[_register_name((2,address))] << 8) | raw[_register_name((2,address+1))]
                gyro[axis] = word - 0x10000 if word & 0x8000 else word
            pll = raw["bank1:0x28"]
            pll = pll - 256 if pll & 128 else pll
            self._select(0)
            return {"raw_registers":raw, "accel_factory_offset":accel,
                    "gyro_user_offset_signed16":gyro, "timebase_pll_signed8":pll,
                    "offset_registers_written":False, "self_test_executed":False,
                    "note":"Raw trim values are not a measured bias or proof of calibration."}
        except BaseException as error:
            self._restore_after_error(error)
            raise

    def _restore_after_error(self, error):
        try:
            self.restore()
        except BaseException as restore_error:
            raise RestoreError(["operation failed: %r" % error,
                                "restore failed: %r" % restore_error]) from error

    def restore(self):
        """Best-effort restore all modified registers; raise on any failure.

        Successful register restores are removed from the pending set. Failed
        ones remain available for an explicit retry before close.
        """
        self._started = False
        if self._bus is None or self._original_bank is None:
            return
        failures = []
        # An earlier partial restore may already have put the chip to sleep or
        # re-enabled LP_EN. Wake it again before retrying bank2/interrupt writes.
        if self._touched and PWR_MGMT_1 in self._original:
            try:
                awake = (self._original[PWR_MGMT_1] & 0x18) | 1
                if self._read(PWR_MGMT_1) != awake:
                    self._touched.add(PWR_MGMT_1)
                    self._write_verified(PWR_MGMT_1, awake)
                    self._sleep(0.050)
            except BaseException as error:
                failures.append("wake before restore: %r" % error)
        for key in _RESTORE_ORDER:
            if key not in self._touched:
                continue
            try:
                self._write_verified(key, self._original[key])
                self._touched.remove(key)
            except BaseException as error:
                failures.append("%s: %r" % (_register_name(key), error))
        try:
            self._bus.write(self.address, REG_BANK_SEL, bytes([self._original_bank]))
            if self._bus_read(REG_BANK_SEL)[0] & 0x30 != self._original_bank:
                raise IMUError("original bank readback mismatch")
        except BaseException as error:
            failures.append("REG_BANK_SEL: %r" % error)
        self.restore_status = "failed" if failures else "restored"
        if failures:
            raise RestoreError(failures)

    def close(self):
        if self._closed:
            return
        try:
            self.restore()
        finally:
            if self._owns_bus and self._bus is not None:
                self._bus.close()
            self._closed = True

    def __enter__(self):
        try:
            self.start()
        except BaseException:
            # start has already attempted restoration; context entry failure
            # must also release an owned adapter, without masking that failure.
            if self._owns_bus and self._bus is not None:
                self._bus.close()
            self._closed = True
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

"""One finite current-position hold with a permanently installed fixed catch.

Terminal cues and acknowledgments record the operator's action. They cannot
prove contact or weight transfer; video and a post-run physical review do that.
"""
import errno
import os
import termios
import time

from .policy_live_profile import fixed_catch_current_hold_settings


class FixedCatchExecution:
    def __init__(self, tty_fd, *, write_fd=None, clock=time.monotonic_ns, close_fd=False):
        if type(tty_fd) is not int or not os.isatty(tty_fd):
            raise ValueError('A visible local Jetson terminal is required')
        if write_fd is None:
            write_fd = tty_fd
        if (type(write_fd) is not int or not os.isatty(write_fd) or
                os.fstat(tty_fd).st_rdev != os.fstat(write_fd).st_rdev):
            raise ValueError('Terminal input and output must be the same visible device')
        self.tty_fd = tty_fd
        self.write_fd = write_fd
        self.close_fd = close_fd
        self.clock = clock
        self.was_blocking = os.get_blocking(tty_fd)
        os.set_blocking(tty_fd, False)
        self.cancel = None
        self.started_ns = None
        self.cue_ns = None
        self.ack_ns = None
        self.last_cycle_ns = None
        self.events = []
        self.input_buffer = b''
        self.warned = False
        self.closed = False
        self.stop_at_ns = None

    def bind_profile(self, profile, *, active):
        catch = fixed_catch_current_hold_settings(profile)
        if catch is None or profile['duration_s'] != 30:
            raise ValueError('Exact reviewed thirty-second fixed-catch hold required')
        if not active:
            return
        self.catch = catch
        reserve = max(axis['max_command_velocity_rad_s'] /
                      axis['max_command_acceleration_rad_s2']
                      for axis in profile['axes'].values()) + profile['stop_duration_s'] + .04
        self.stop_at_ns = int((profile['duration_s'] - reserve) * 1e9)

    def connect_cancel(self, cancel):
        self.cancel = cancel

    def wrap_model(self, model):
        # Fixed current hold only calls validate_inputs on the original model.
        return model

    def _write(self, message, key):
        raw = message.encode('utf-8')
        try:
            count = os.write(self.write_fd, raw)
        except (BlockingIOError, OSError) as error:
            self._failure('TERMINAL_CUE_FAILED:' + type(error).__name__)
            raise RuntimeError('Fixed-catch terminal cue failed') from error
        if count != len(raw):
            self._failure('TERMINAL_CUE_PARTIAL')
            raise RuntimeError('Fixed-catch terminal cue was incomplete')
        stamp = self.clock()
        self.events.append({'key': key, 'monotonic_ns': stamp})
        return stamp

    def _failure(self, reason):
        self.events.append({'key': reason, 'monotonic_ns': self.clock()})
        if self.cancel is not None:
            self.cancel()

    def _poll_input(self):
        # A bounded nonblocking read keeps the 20 ms control cycle independent
        # of human timing and terminal readiness. Input before the cue is stale.
        try:
            chunk = os.read(self.tty_fd, 128)
        except BlockingIOError:
            return
        except OSError as error:
            self._failure('TERMINAL_READ_FAILED:' + type(error).__name__)
            raise RuntimeError('Fixed-catch terminal disappeared') from error
        if not chunk:
            self._failure('TERMINAL_CLOSED')
            raise RuntimeError('Fixed-catch terminal closed')
        self.input_buffer += chunk
        if len(self.input_buffer) > 256:
            self._failure('TERMINAL_INPUT_TOO_LONG')
            raise RuntimeError('Fixed-catch terminal input too long')
        while b'\n' in self.input_buffer:
            line, self.input_buffer = self.input_buffer.split(b'\n', 1)
            word = line.strip().lower()
            if word in (b'q', b'stop'):
                self._failure('OPERATOR_STOP')
                raise RuntimeError('Operator requested immediate stop')
            if self.cue_ns is not None and self.ack_ns is None and word in (b'', b'done'):
                self.ack_ns = self.clock()
                self.events.append({'key': 'UPPER_SUPPORT_REMOVAL_ACK', 'monotonic_ns': self.ack_ns})

    def on_start(self, started_ns):
        if self.started_ns is not None:
            raise RuntimeError('Fixed-catch hold cannot restart')
        self.started_ns = started_ns
        self._write('\n固定受けを残してください。高い箱は合図まで抜きません。\n', 'HOLD_STARTED')

    def before_cycle(self, begun_ns, *, stop_requested=False):
        if self.started_ns is None:
            raise RuntimeError('Fixed-catch hold has not started')
        self._poll_input()
        elapsed_ns = begun_ns - self.started_ns
        if self.cue_ns is not None and self.ack_ns is None and elapsed_ns >= 8_000_000_000:
            self.events.append({'key': 'NO_REMOVAL_ACK_BY_8S', 'monotonic_ns': begun_ns})
            return True
        return bool(stop_requested or
                    self.stop_at_ns is not None and elapsed_ns >= self.stop_at_ns)

    def after_cycle_validated(self, begun_ns, ended_ns, phase, *, stop_requested=False):
        if self.started_ns is None:
            raise RuntimeError('Fixed-catch hold has not started')
        self.last_cycle_ns = ended_ns
        elapsed_ns = ended_ns - self.started_ns
        if (self.cue_ns is None and phase == 'active' and elapsed_ns >= 2_000_000_000
                and not stop_requested):
            # Drain old Enter presses before opening the acknowledgment window.
            self._poll_input()
            self.input_buffer = b''
            try:
                termios.tcflush(self.tty_fd, termios.TCIFLUSH)
            except OSError as error:
                self._failure('TERMINAL_FLUSH_FAILED:' + type(error).__name__)
                raise RuntimeError('Fixed-catch input boundary failed') from error
            self.cue_ns = self._write(
                '\nFIXED_CATCH_WINDOW_OPEN: 受けを残し高い箱だけ抜いてください。'
                '完了したらEnter。8秒までに完了できなければ保持を終了します。\n',
                'UPPER_SUPPORT_WITHDRAWAL_CUE')
        if self.cue_ns is not None and not self.warned and elapsed_ns >= 26_000_000_000:
            self._write('\nSTOPまで約3秒。低い固定受けをそのまま残してください。\n',
                        'STOP_APPROACHING')
            self.warned = True

    def decorate_report(self, report):
        report['fixed_catch_trial'] = {
            'scope': 'fixed_catch_current_hold_only',
            'recovery_basis': 'passive_fixed_catch_prearmed',
            'cue_ns': self.cue_ns,
            'upper_support_removal_ack_ns': self.ack_ns,
            'actual_box_removal_or_weight_transfer_measured': False,
            'contact_and_stance_operator_review_required': True,
            'events': list(self.events),
        }
        return report

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            os.set_blocking(self.tty_fd, self.was_blocking)
        except OSError:
            pass
        if self.close_fd:
            os.close(self.tty_fd)

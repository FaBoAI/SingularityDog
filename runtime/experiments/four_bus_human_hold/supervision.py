"""Bounded local-terminal supervision for a separately prepared new 30 Hz hold.

Derived from the reviewed legacy human-supported finite supervisor. The old
runner/profile are untouched. Only a distinct profile proof may bind actively;
this module opens no CAN or motor device. Whole-cycle/reply budget is 30 Hz;
all-axis host-write and cue freshness remain 20 ms. Operator Enter never proves
load transfer or contact. Emergency cancellation does not wait for Enter.
"""
import math
from contextlib import contextmanager
import os
import termios
import threading
import time

from .profile import SCOPE, SETTINGS, human_supported_hold_settings, supervisor_view

PERIOD_NS = 33_333_334
INPUT_TX_NS = 20_000_000
MAX_CUE_AGE_NS = 20_000_000


@contextmanager
def linux_timer_scope():
    """Pin the independent recovery timer away from policy CPU4, restore it."""
    if not all(callable(getattr(os,name,None)) for name in ('sched_getaffinity','sched_setaffinity')):
        raise RuntimeError('Explicit Linux timer affinity support required')
    original=sorted(os.sched_getaffinity(0))
    if not original:
        raise RuntimeError('Timer owner affinity was empty')
    tid=threading.get_native_id()
    record={'native_tid':tid,'before_cpu_mask':original,'cpu_mask':None,
            'file_only_mock_readback':False,'restored':False}
    try:
        os.sched_setaffinity(0,{0,1,2,3})
        record['cpu_mask']=sorted(os.sched_getaffinity(0))
        if record['cpu_mask'] != [0,1,2,3]:
            raise RuntimeError('Recovery timer CPU0..3 readback differs')
        yield record
    finally:
        os.sched_setaffinity(0,set(original))
        record['after_cpu_mask']=sorted(os.sched_getaffinity(0))
        record['restored']=record['after_cpu_mask']==original
        if not record['restored']:
            raise RuntimeError('Recovery timer affinity restoration differs')


class HumanSupported30Execution:
    """Fresh validated-cycle cue, independent 0.5s timer and fresh full-support ACK.

    Integration must call on_abort before emergency STOP and before_stop before
    normal gain-down. Exact-class admission belongs to the live loader/runtime;
    this standalone class is deliberately not an executable trial entry point.
    """
    def __init__(self, tty_fd, *, write_fd=None, clock=time.monotonic_ns,
                 close_fd=False, thread_factory=threading.Thread, stage_audio=None,
                 timer_scope=linux_timer_scope):
        if type(tty_fd) is not int or not os.isatty(tty_fd):
            raise ValueError('A visible local Jetson terminal is required')
        write_fd = tty_fd if write_fd is None else write_fd
        if (type(write_fd) is not int or not os.isatty(write_fd) or
                os.fstat(tty_fd).st_rdev != os.fstat(write_fd).st_rdev):
            raise ValueError('Input/output must use the same visible terminal')
        if not callable(timer_scope):
            raise ValueError('Explicit recovery timer operating scope required')
        self.tty_fd, self.write_fd = tty_fd, write_fd
        self.clock, self.close_fd = clock, close_fd
        self._original = {fd: os.get_blocking(fd) for fd in (tty_fd, write_fd)}
        try:
            for fd in self._original:
                os.set_blocking(fd, False)
        except BaseException:
            for fd, state in self._original.items():
                try: os.set_blocking(fd, state)
                except OSError: pass
            raise
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._closed = threading.Event()
        self._thread_factory = thread_factory
        self._timer_scope=timer_scope
        self.timer_readback=None
        self.restored=False
        self.restoration_report=None
        self._genuine_primary=False
        self._timer = None
        self._settings = None
        self._audio_manifest = None
        self.stage_audio = stage_audio
        self._active_bound = False
        self.cancel = None
        self.started_ns = self.cue_ns = self.resupport_cue_ns = self.ack_ns = None
        self.ease_deadline_ns = self.ack_deadline_ns = self.latest_stop_ns = None
        self.prepare_request_ns = self.prepare_finished_ns = None
        self.go_request_ns = self.go_finished_ns = None
        self.resupport_audio_finished_ns = self.ack_prompt_ns = None
        self.last_full_gain_ns = None
        self.audio_requests = {}
        self.audio_starts = {}
        self.audio_completions = {}
        self.recovery_audio_requested = False
        self.last_cycle_ns = None
        self.events = []
        self.input_buffer = b''
        self.failed = None
        self.skipped = False
        self.stopping = False
        self.closed = False

    def bind_profile(self, profile, *, active):
        if self.closed or self.started_ns is not None:
            raise RuntimeError('Cannot rebind a started human-supported hold')
        profile, manifest = supervisor_view(profile, active=active)
        self._genuine_primary = active and profile.get('_genuine_primary_execution_proof') is True
        if self._genuine_primary and self._timer_scope is not linux_timer_scope:
            raise ValueError('Genuine human execution requires the source-owned Linux recovery timer scope')
        settings = human_supported_hold_settings(profile)
        if settings is None:
            raise ValueError('Human-supported current-hold contract is absent')
        if type(active) is not bool:
            raise ValueError('Explicit active selection required')
        if type(manifest) is not dict or type(manifest.get('clips')) is not dict:
            raise ValueError('A token-bound reviewed audio manifest is required')
        clips = manifest['clips']
        if set(clips) != {'brief', 'prepare_ease', 'go', 'resupport', 'abort'}:
            raise ValueError('All five reviewed audio clips are required')
        durations = {}
        for stage, clip in clips.items():
            duration = clip.get('duration_s') if type(clip) is dict else None
            if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
                raise ValueError('Measured audio duration is required: ' + stage)
            durations[stage] = duration
        if durations['go'] > .12:
            raise ValueError('Go cue must be a reviewed short sound, at most 0.12s')
        if (SETTINGS['cue_not_before_s'] + durations['prepare_ease'] + .25 +
                durations['go'] + .25 + SETTINGS['slight_ease_max_duration_s'] +
                durations['resupport'] + .25 + SETTINGS['resupport_ack_window_s'] + PERIOD_NS/1e9 >
                settings['latest_stop_s']):
            raise ValueError('Reviewed audio, slight easing and return reserve do not fit')
        if active and (self.stage_audio is None or any(not callable(getattr(self.stage_audio, name, None))
                                                     for name in ('start', 'cancel', 'close'))):
            raise ValueError('A prestarted nonblocking owned audio worker is required')
        self._audio_manifest = manifest
        self._settings, self._active_bound = settings, active

    def connect_cancel(self, cancel):
        if not callable(cancel):
            raise ValueError('Emergency cancellation callback is required')
        self.cancel = cancel

    def wrap_model(self, model):
        # Preserve the original model's complete IMU/input validation; never infer.
        return model

    def _event(self, key, stamp=None):
        if len(self.events) >= 64:
            raise RuntimeError('Human-supported event limit exceeded')
        self.events.append({'key': key, 'monotonic_ns': self.clock() if stamp is None else stamp})

    def _write(self, text, key):
        if self._closed.is_set():
            raise RuntimeError('Human-supported terminal supervisor is closed')
        raw = text.encode('utf-8')
        count = os.write(self.write_fd, raw)
        if count != len(raw):
            raise RuntimeError('Incomplete human-supported terminal cue')
        stamp = self.clock()
        self._event(key, stamp)
        return stamp

    def _resupport(self, *, no_ease=False):
        if self.resupport_cue_ns is not None:
            return
        text = ('NO_PARTIAL_LOAD ' if no_ease else '') + (
            'RE_SUPPORT_NOW: 手を離さず胴体を全支持してください。実際に全支持してからEnter。\n')
        self.resupport_cue_ns = self._write(text, 'FULL_SUPPORT_REQUIRED')
        if self.failed is not None or no_ease:
            self._recovery_audio()
        else:
            self._start_audio('resupport')

    def _recovery_audio(self):
        if self.stage_audio is None or self.recovery_audio_requested or self._closed.is_set():
            return
        self.recovery_audio_requested = True
        self.stage_audio.cancel()
        self._start_audio('abort')

    def _start_audio(self, stage):
        if stage in self.audio_requests:
            raise RuntimeError('An audio stage cannot repeat: ' + stage)
        self.audio_requests[stage] = self.clock()
        self._event('AUDIO_REQUEST_' + stage.upper(), self.audio_requests[stage])
        self.stage_audio.start(stage, self._audio_complete, self._audio_failure,
                               on_started=self._audio_started)

    def _audio_started(self, stage, started_ns):
        with self._lock:
            if self._closed.is_set() or (self.failed is not None and stage != 'abort'):
                return
            try:
                now = self.clock()
                request = self.audio_requests.get(stage)
                if (request is None or stage in self.audio_starts or type(started_ns) is not int or
                        not request <= started_ns <= now or now - started_ns > MAX_CUE_AGE_NS):
                    raise RuntimeError('Noncausal or delayed audio start: ' + stage)
                self.audio_starts[stage] = started_ns
                self._event('AUDIO_STARTED_' + stage.upper(), started_ns)
                if stage == 'go':
                    if (self.last_full_gain_ns is None or now - self.last_full_gain_ns > MAX_CUE_AGE_NS or
                            self.prepare_finished_ns is None or self.skipped or self.stopping or
                            self.resupport_cue_ns is not None or now > self._latest_go_start_ns()):
                        raise RuntimeError('Go audio lacks fresh full-gain evidence/recovery reserve')
                    self.cue_ns = started_ns
                    self.ease_deadline_ns = started_ns + 500_000_000
                    self._event('SLIGHT_EASE_WINDOW_OPEN', started_ns)
                    self._write('PARTIAL_LOAD_WINDOW_OPEN: 開始音を合図に手を離さず少しだけ緩め、'
                                '声を待たず0.5秒以内に全支持へ戻してください。\n',
                                'SLIGHT_EASE_CUE_DISPLAYED')
                    self._wake.set()
            except BaseException:
                self._fail('HUMAN_SUPPORTED_AUDIO_START_FAILED')

    def _audio_complete(self, stage, started_ns, finished_ns):
        with self._lock:
            if self._closed.is_set() or (self.failed is not None and stage != 'abort'):
                return
            try:
                now = self.clock()
                request = self.audio_requests.get(stage)
                maximum = self._audio_manifest['clips'][stage]['duration_s'] + .25
                if (request is None or stage in self.audio_completions or
                        self.audio_starts.get(stage) != started_ns or
                        type(finished_ns) is not int or not started_ns <= finished_ns <= now or
                        finished_ns - request > round(maximum * 1e9)):
                    raise RuntimeError('Unconfirmed or late owned audio completion: ' + stage)
                self.audio_completions[stage] = finished_ns
                self._event('AUDIO_COMPLETED_' + stage.upper(), finished_ns)
                if stage == 'prepare_ease':
                    self.prepare_finished_ns = finished_ns
                elif stage == 'go':
                    self.go_finished_ns = finished_ns
                elif stage == 'resupport':
                    # Enter before voice completion remains stale.
                    self._poll_input(accept_ack=False)
                    self.input_buffer = b''
                    termios.tcflush(self.tty_fd, termios.TCIFLUSH)
                    self.resupport_audio_finished_ns = finished_ns
                    self.ack_prompt_ns = self._write('全支持音声終了。実際に全支持してからEnter。\n',
                                                     'FRESH_FULL_SUPPORT_ACK_REQUIRED')
                    self.ack_deadline_ns = self.ack_prompt_ns + 500_000_000
                    if self.ack_deadline_ns + PERIOD_NS > self.latest_stop_ns:
                        raise RuntimeError('Audio completion left insufficient full-support ACK reserve')
            except BaseException:
                self._fail('HUMAN_SUPPORTED_AUDIO_COMPLETION_FAILED')

    def _audio_failure(self, stage, reason):
        with self._lock:
            if self._closed.is_set(): return
            self._event('AUDIO_FAILED_' + str(stage).upper())
            self._fail('HUMAN_SUPPORTED_AUDIO_FAILURE:' + str(reason)[:120])

    def _latest_go_start_ns(self):
        after_go_s = .5 + self._audio_manifest['clips']['resupport']['duration_s'] + .25 + .5 + PERIOD_NS/1e9
        return self.latest_stop_ns - round(after_go_s * 1e9)

    def _fail(self, reason):
        if self.failed is None:
            self.failed = reason
            self._event(reason)
            if self._closed.is_set():
                # Do not touch restored blocking fds or another owner's cancelled
                # resources after cleanup. This object can never be rearmed.
                return
            try: self._resupport(no_ease=self.cue_ns is None)
            except BaseException:
                # A disappearing/full terminal cannot delay existing emergency STOP.
                self._event('RECOVERY_CUE_UNCONFIRMED')
            try: self._recovery_audio()
            except BaseException: self._event('RECOVERY_AUDIO_UNCONFIRMED')
            self._wake.set()
            if self.cancel is not None:
                self.cancel()

    def _poll_input(self, *, accept_ack=True):
        try: chunk = os.read(self.tty_fd, 128)
        except BlockingIOError: return
        if not chunk:
            raise RuntimeError('Human-supported terminal closed')
        self.input_buffer += chunk
        if len(self.input_buffer) > 256:
            raise RuntimeError('Human-supported terminal input too long')
        while b'\n' in self.input_buffer:
            line, self.input_buffer = self.input_buffer.split(b'\n', 1)
            word = line.strip().lower()
            if word in (b'q', b'stop'):
                raise InterruptedError('Operator requested immediate stop')
            if accept_ack and word in (b'', b'resupported') and self.ack_ns is None:
                stamp = self.clock()
                if (self.ack_prompt_ns is not None and self.ack_deadline_ns is not None and
                        self.ack_prompt_ns < stamp < self.ack_deadline_ns and self.failed is None):
                    self.ack_ns = stamp
                    self._event('OPERATOR_FULL_SUPPORT_ACK', stamp)

    def on_start(self, started_ns):
        with self._lock:
            if self.closed or self.started_ns is not None:
                raise RuntimeError('Human-supported hold cannot restart')
            if not self._active_bound or self.cancel is None:
                raise RuntimeError('Reviewed active binding and cancellation are required')
            if type(started_ns) is not int or not 0 < started_ns <= self.clock():
                raise ValueError('Causal start timestamp is required')
            self.started_ns = started_ns
            self.latest_stop_ns = started_ns + round(self._settings['latest_stop_s'] * 1e9)
            try:
                self._write('胴体を全支持し続けてください。手は離さず、正常保持の合図まで力を緩めません。\n',
                            'HUMAN_FULL_SUPPORT_STARTED')
                self._timer = self._thread_factory(target=self._timer_main,
                                                  name='human-hold-cue-timer', daemon=True)
                self._timer.start()
            except BaseException:
                self._fail('TERMINAL_OR_TIMER_START_FAILED')
                raise

    def _timer_main(self):
        try:
            with self._timer_scope() as record:
                if not isinstance(record,dict) or record.get('cpu_mask') != [0,1,2,3]:
                    raise RuntimeError('Recovery timer scope requires CPU0..3 readback')
                self.timer_readback=record
                self._timer_body()
        except BaseException:
            with self._lock:
                self._fail('INDEPENDENT_RECOVERY_TIMER_SCOPE_FAILED')

    def _timer_body(self):
        # Sleep until one bounded cue exists; no periodic GIL wakeups during gain-up.
        self._wake.wait()
        while not self._closed.is_set():
            with self._lock:
                if self.failed is not None or self.ease_deadline_ns is None or self.resupport_cue_ns is not None:
                    return
                left_s = (self.ease_deadline_ns - self.clock()) / 1e9
            if left_s > 0 and not self._closed.wait(left_s):
                continue
            if self._closed.is_set(): return
            try:
                with self._lock: self._check_deadlines(self.clock())
            except BaseException:
                with self._lock: self._fail('INDEPENDENT_RESUPPORT_TIMER_FAILED')
            return

    def _check_deadlines(self, now_ns):
        if self.failed is not None:
            raise RuntimeError(self.failed)
        for stage, request in self.audio_requests.items():
            if stage != 'abort' and stage not in self.audio_completions:
                timeout_ns = round((self._audio_manifest['clips'][stage]['duration_s'] + .25) * 1e9)
                if now_ns >= request + timeout_ns:
                    self._fail('OWNED_AUDIO_COMPLETION_TIMEOUT:' + stage)
                    raise RuntimeError('Owned audio did not complete within its reviewed bound')
        if self.ease_deadline_ns is not None and now_ns >= self.ease_deadline_ns:
            self._resupport()
        if (self.ack_deadline_ns is not None and now_ns >= self.ack_deadline_ns and
                self.ack_ns is None):
            self._fail('FULL_SUPPORT_ACK_DEADLINE_EXPIRED')
            raise RuntimeError('Full support was not freshly acknowledged before STOP reserve')
        if now_ns >= self.latest_stop_ns and self.cue_ns is not None and self.ack_ns is None:
            self._fail('FULL_SUPPORT_UNCONFIRMED_AT_SHUTDOWN_RESERVE')
            raise RuntimeError('Full support was not confirmed before shutdown reserve')

    def before_cycle(self, begun_ns, *, stop_requested=False):
        with self._lock:
            if self.closed or self.started_ns is None:
                raise RuntimeError('Human-supported hold has not started')
            try:
                if type(begun_ns) is not int or not self.started_ns <= begun_ns <= self.clock():
                    raise ValueError('Causal cycle start timestamp is required')
                self._check_deadlines(self.clock())
                self._poll_input()
                if stop_requested and self.cue_ns is None:
                    self.stopping = True
                    self.skipped = True
                    self.stage_audio.cancel()
                    self._resupport(no_ease=True)
                    return True  # No easing occurred; continuous full support was required.
                if stop_requested and self.resupport_cue_ns is None:
                    self._resupport()
                return bool(self.ack_ns is not None or self.skipped)
            except BaseException:
                self._fail('HUMAN_SUPPORTED_CYCLE_SUPERVISION_FAILED')
                raise

    def after_cycle_validated(self, begun_ns, ended_ns, phase, *, stop_requested=False,
                              full_gain=False, oldest_input_ns, final_write_ns, last_reply_ns):
        with self._lock:
            if self.closed or self.started_ns is None:
                raise RuntimeError('Human-supported hold has not started')
            try:
                now = self.clock()
                self._check_deadlines(now)
                if (type(begun_ns) is not int or type(ended_ns) is not int or
                        not self.started_ns <= begun_ns <= ended_ns <= now or
                        ended_ns - begun_ns > PERIOD_NS or now - ended_ns > MAX_CUE_AGE_NS or
                        self.last_cycle_ns is not None and begun_ns <= self.last_cycle_ns):
                    raise RuntimeError('Fresh complete validated holding-cycle evidence is required')
                stamps = (oldest_input_ns, final_write_ns, last_reply_ns)
                if (any(type(value) is not int or value <= 0 for value in stamps) or
                        not begun_ns <= oldest_input_ns <= final_write_ns <= last_reply_ns <= ended_ns or
                        final_write_ns >= min(begun_ns, oldest_input_ns) + INPUT_TX_NS or
                        last_reply_ns >= begun_ns + PERIOD_NS):
                    raise RuntimeError('Fresh 20ms all-axis writes and 30Hz replies are required')
                self.last_cycle_ns = ended_ns
                if type(full_gain) is not bool:
                    raise ValueError('Explicit all-axis full-gain validation is required')
                self.last_full_gain_ns = ended_ns if full_gain and phase == 'active' else None
                if (self.cue_ns is not None or self.skipped or self.stopping or stop_requested or
                        phase != 'active' or not full_gain):
                    return
                if ended_ns < self.started_ns + round(self._settings['cue_not_before_s'] * 1e9):
                    return
                latest_cue = self._latest_go_start_ns()
                remaining = (self._audio_manifest['clips']['prepare_ease']['duration_s'] + .25
                             if self.prepare_request_ns is None else 0.)
                if now + round(remaining * 1e9) > latest_cue:
                    self.skipped = True
                    self._resupport(no_ease=True)
                    return
                if self.prepare_request_ns is None:
                    self.prepare_request_ns = now
                    self._start_audio('prepare_ease')
                elif (self.prepare_finished_ns is not None and begun_ns > self.prepare_finished_ns and
                      self.go_request_ns is None):
                    self.go_request_ns = now
                    self._start_audio('go')
            except BaseException:
                self._fail('HUMAN_SUPPORTED_VALIDATED_CYCLE_FAILED')
                raise

    def before_stop(self, *, emergency=False):
        """Normal gain-down needs full support; emergency STOP never waits."""
        with self._lock:
            if emergency:
                self.on_abort('EMERGENCY_STOP_STARTED')
                return
            if self.failed is not None:
                raise RuntimeError('A failed human-supported trial cannot stop normally')
            if self.cue_ns is not None and self.ack_ns is None:
                self._fail('NORMAL_STOP_WITHOUT_FULL_SUPPORT_ACK')
                raise RuntimeError('Fresh full-support acknowledgement is required before gain-down')
            self.stopping = True
            if self.cue_ns is None:
                self.skipped = True
                self.stage_audio.cancel()
                self._resupport(no_ease=True)
            self._event('FULL_SUPPORT_BEFORE_NORMAL_STOP')

    def on_abort(self, reason='HUMAN_SUPPORTED_RUNTIME_ABORT'):
        with self._lock:
            self._fail(reason)

    def decorate_report(self, report):
        with self._lock:
            report['human_supported_trial'] = {
                'scope': SCOPE,
                'recovery_basis': 'two_operators_continuous_human_body_catch',
                'slight_ease_cue_ns': self.cue_ns,
                'slight_ease_deadline_ns': self.ease_deadline_ns,
                'full_support_cue_ns': self.resupport_cue_ns,
                'full_support_ack_ns': self.ack_ns,
                'ack_deadline_ns': self.ack_deadline_ns,
                'audio_requests': dict(self.audio_requests),
                'audio_started': dict(self.audio_starts),
                'audio_completed': dict(self.audio_completions),
                'prepare_audio_finished_ns': self.prepare_finished_ns,
                'go_audio_finished_ns': self.go_finished_ns,
                'resupport_audio_finished_ns': self.resupport_audio_finished_ns,
                'audibility_and_physical_ease_duration_measured': False,
                'failure': self.failed,
                'easing_skipped': self.skipped,
                'events': list(self.events),
                'recovery_timer_operating_readback': self.timer_readback,
                'restoration_complete': self.restored,
                'restoration': self.restoration_report,
                'actual_full_support_or_weight_transfer_measured': False,
                'contact_and_stance_operator_review_required': True,
                'standing_or_walking_authorized': False,
            }
            return report

    def close(self):
        if self.closed:
            return self.restoration_report
        # closed prevents rebinding; restored is a separate verified outcome.
        self.closed = True
        report = self.restoration_report = {'terminal': {}, 'timer': {}, 'audio': {},
            'errors': [], 'complete': False}
        def failed(stage,error):
            report['errors'].append({'stage': stage, 'error': type(error).__name__+': '+str(error)})
            with self._lock:
                self._fail('HUMAN_SUPPORTED_'+stage.upper()+'_RESTORATION_UNCONFIRMED')
        with self._lock:
            if self.cue_ns is not None and self.resupport_cue_ns is None:
                self._fail('SUPERVISOR_CLOSED_DURING_EASING')
        self._closed.set()
        self._wake.set()
        try:
            if self._timer is not None and self._timer.is_alive():
                self._timer.join(timeout=.2)
            alive=self._timer is not None and self._timer.is_alive()
            record=self.timer_readback
            timer_good=(not alive and (not self._genuine_primary or
                isinstance(record,dict) and record.get('restored') is True and
                record.get('cpu_mask') == [0,1,2,3] and record.get('file_only_mock_readback') is False))
            report['timer']={'thread_alive':bool(alive),'readback':record,'restored':bool(timer_good)}
            if not timer_good:
                raise RuntimeError('Timer thread/affinity restoration was not confirmed')
        except BaseException as error:
            failed('timer',error)
        try:
            if self.stage_audio is not None:
                self.stage_audio.close()
                if getattr(self.stage_audio,'closed',False) is not True:
                    raise RuntimeError('Owned audio worker did not close')
            report['audio']={'closed':True}
        except BaseException as error:
            report['audio']={'closed':False}
            failed('audio',error)
        for fd,state in self._original.items():
            try:
                os.set_blocking(fd,state)
                actual=os.get_blocking(fd)
                report['terminal'][str(fd)]={'original_blocking':state,'after_blocking':actual,
                    'restored':actual is state}
                if actual is not state:
                    raise RuntimeError('Terminal blocking-mode restoration readback differed')
            except BaseException as error:
                report['terminal'].setdefault(str(fd),{'original_blocking':state,'restored':False})
                failed('terminal',error)
        if self.close_fd:
            try:
                os.close(self.tty_fd)
                report['owned_input_fd_closed']=True
            except BaseException as error:
                report['owned_input_fd_closed']=False
                failed('terminal_close',error)
        self.restored=(not report['errors'] and report['timer'].get('restored') is True and
            report['audio'].get('closed') is True and
            all(row.get('restored') is True for row in report['terminal'].values()))
        report['complete']=self.restored
        return report

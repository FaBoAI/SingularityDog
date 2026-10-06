"""Bounded camera-only JPEG recording; default PLAN never imports GI or opens devices.

Host brackets describe appsink receipt, not exposure. Gst PTS/DTS/offset values
are retained without conversion to host time or claims about lost camera frames.
Explicit recording creates a fresh private directory outside Git. Partial JPEGs
and failure receipts are retained; there is no retry, CAN, audio or motor output.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import time

SCHEMA = 'singularitydog.usb-camera-motion-capture.v1'
DEVICE = None  # Actual serial-bearing by-id paths belong in private invocations.
WIDTH, HEIGHT, FPS = 1920, 1080, 30
MAX_FRAME_BYTES = 16 * 1024 * 1024
FLAGS = dict.fromkeys(('can_opened', 'serial_opened', 'audio_opened', 'motor_output_allowed',
    'approved_for_runtime', 'calibration_approved', 'physical_stationarity_proven',
    'velocity_cause_confirmed', 'exposure_time_verified', 'clock_alignment_verified',
    'uncertainty_bounds_verified', 'camera_frame_loss_count_verified', 'jpeg_decode_verified', 'automatic_retry'), False)


def need(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def make_plan(device=DEVICE, seconds=10.0, frames=300, max_bytes=128 * 1024 * 1024):
    need(device is None or (type(device) is str and re.fullmatch(r'/dev/v4l/by-id/[A-Za-z0-9_.:-]+-video-index0', device)),
         'Select an absolute stable by-id video-index0 path')
    need(type(seconds) in (int, float) and math.isfinite(seconds) and .1 <= seconds <= 30,
         'Duration must be finite 0.1..30 seconds')
    need(type(frames) is int and 1 <= frames <= 900, 'Frame cap must be 1..900')
    need(type(max_bytes) is int and 1024 <= max_bytes <= 512 * 1024 * 1024,
         'JPEG byte cap must be 1024..536870912')
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', **FLAGS, 'camera_opened': False,
        'device': device, 'seconds': float(seconds), 'frames': frames, 'max_bytes': max_bytes,
        'max_frame_bytes': MAX_FRAME_BYTES, 'requested_caps': {'media_type': 'image/jpeg',
            'width': WIDTH, 'height': HEIGHT, 'fps_num': FPS, 'fps_den': 1},
        'host_timestamp_scope': 'Monotonic brackets around appsink receipt; not exposure or sensor time.',
        'gst_timestamp_scope': 'Uninterpreted stream timestamps/offsets; no host clock conversion.',
        'device_path_supplied': device is not None, 'startup_wait_cap_ns': 2_000_000_000,
        'startup_consumes_capture_duration': True,
        'frame_loss_count': None, 'exposure_s': None, 'timestamp_max_error_s': None,
        'external_clock_alignment': None, 'physical_angle_resolution_rad': None,
        'stop_scope': 'Frame/duration/byte bounds; state cleanup has no hard wall-time guarantee.'}


def signature(info):
    need(stat.S_ISCHR(info.st_mode), 'Camera endpoint must be a character device')
    return {'st_dev': info.st_dev, 'st_ino': info.st_ino, 'st_rdev': info.st_rdev}


def device_binding(device):
    path = Path(device)
    need(not any(p.is_symlink() for p in path.parents), 'Camera path parent symlink refused')
    link = path.lstat()
    need(stat.S_ISLNK(link.st_mode), 'Stable camera by-id entry must be a symlink')
    resolved = path.resolve(strict=True)
    need(re.fullmatch(r'/dev/video[0-9]+', str(resolved)), 'Unexpected camera symlink target')
    return {'by_id': str(path), 'resolved': str(resolved),
            'link_identity': {'st_dev': link.st_dev, 'st_ino': link.st_ino},
            'device_identity': signature(resolved.stat())}


def regular_source():
    path = Path(__file__).absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)) and path.is_file(),
         'Regular nonsymlink recorder source required')
    return digest(path.read_bytes())


def private_output(path):
    path = Path(path).expanduser().absolute()
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Output symlink refused')
    need(path.parent.is_dir() and not path.exists(), 'Fresh output with existing parent required')
    need(not any((p/'.git').exists() for p in (path, *path.parents)), 'Private output must be outside Git')
    path.mkdir(mode=0o700)
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0))
    os.fchmod(fd, 0o700)
    return path, fd


def check_output(path, fd):
    need(not any(p.is_symlink() for p in (path, *path.parents)), 'Output binding became symlinked')
    a, b = path.stat(), os.fstat(fd)
    need(stat.S_ISDIR(a.st_mode) and (a.st_dev, a.st_ino) == (b.st_dev, b.st_ino),
         'Output directory identity changed')


def write_member(fd, name, raw):
    need(type(name) is str and '/' not in name and name not in ('', '.', '..'), 'Flat output name required')
    out = os.open(name, os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600, dir_fd=fd)
    with os.fdopen(out, 'w+b') as stream:
        os.fchmod(stream.fileno(), 0o600)
        stream.write(raw)
        stream.flush(); os.fsync(stream.fileno())
        stream.seek(0)
        saved = stream.read(len(raw)+1)
        info = os.fstat(stream.fileno())
        named = os.stat(name, dir_fd=fd, follow_symlinks=False)
        need(stat.S_ISREG(info.st_mode) and stat.S_ISREG(named.st_mode) and
             (info.st_dev, info.st_ino) == (named.st_dev, named.st_ino) and saved == raw,
             'Saved output member identity/bytes differ: '+name)
    return digest(raw)


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()


def validate_sample(sample):
    expected = {'jpeg', 'actual_caps', 'media_type', 'width', 'height', 'fps_num', 'fps_den',
                'pts_ns', 'dts_ns', 'duration_ns', 'offset', 'offset_end', 'buffer_flags'}
    need(type(sample) is dict and set(sample) == expected, 'Unexpected camera sample fields')
    raw = sample['jpeg']
    need(type(raw) is bytes and 0 < len(raw) <= MAX_FRAME_BYTES, 'Empty/oversized camera payload')
    need(type(sample['actual_caps']) is str and 0 < len(sample['actual_caps']) <= 4096,
         'Missing/bounded actual caps required')
    for key in ('width', 'height', 'fps_num', 'fps_den', 'buffer_flags'):
        need(type(sample[key]) is int and 0 <= sample[key] < 2**32, 'Invalid sample '+key)
    need(type(sample['media_type']) is str, 'Invalid sample media type')
    for key in ('pts_ns', 'dts_ns', 'duration_ns', 'offset', 'offset_end'):
        v = sample[key]
        need(v is None or (type(v) is int and 0 <= v < 2**64-1), 'Invalid Gst metadata '+key)
    return {k: v for k, v in sample.items() if k != 'jpeg'}


def jpeg_markers(raw):
    # UVC may append padding after EOI. Retain original bytes; this structural
    # observation does not prove that an image decoder will accept the JPEG.
    end = raw.rfind(b'\xff\xd9')
    return {'starts_with_soi': raw.startswith(b'\xff\xd8'),
            'last_eoi_byte_offset': None if end < 0 else end,
            'bytes_after_last_eoi': None if end < 0 else len(raw)-end-2,
            'marker_check_only': True, 'decoder_validation_performed': False}


class GstCamera:
    """Only this explicitly constructed backend imports GI; no subprocesses."""
    def __init__(self, plan, binding):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        self.Gst, self.binding = Gst, binding
        self.messages, self.messages_truncated, self.error_seen = [], False, False
        self.opened_device, self.null_confirmed = None, False
        self.pipeline = Gst.parse_launch('v4l2src name=camera device="'+binding['resolved']+'" ! '
            'image/jpeg,width=1920,height=1080,framerate=30/1 ! '
            'appsink name=frames sync=false max-buffers=1 drop=false wait-on-eos=false')
        self.source = self.pipeline.get_by_name('camera')
        self.sink = self.pipeline.get_by_name('frames')
        self.bus = self.pipeline.get_bus()

    def messages_read(self, *, fail=True):
        first_error = None
        types = self.Gst.MessageType.ERROR | self.Gst.MessageType.WARNING | self.Gst.MessageType.EOS
        for _ in range(256):
            msg = self.bus.pop_filtered(types)
            if msg is None:
                break
            kind = 'EOS' if msg.type == self.Gst.MessageType.EOS else (
                'ERROR' if msg.type == self.Gst.MessageType.ERROR else 'WARNING')
            row = {'type': kind, 'source': msg.src.get_name() if msg.src else None}
            if kind != 'EOS':
                error, debug = msg.parse_error() if kind == 'ERROR' else msg.parse_warning()
                row.update(message=str(error)[:4096], debug=None if debug is None else str(debug)[:4096])
            if len(self.messages) < 256:
                self.messages.append(row)
            else:
                self.messages_truncated = True
            if kind == 'ERROR' and first_error is None:
                first_error = row
            if kind == 'ERROR':
                self.error_seen = True
        else:
            self.messages_truncated = True
        if fail and first_error is not None:
            raise RuntimeError('Gst ERROR '+json.dumps(first_error, ensure_ascii=False))

    def check(self):
        self.messages_read()

    def start(self, timeout_ns):
        result = self.pipeline.set_state(self.Gst.State.PLAYING)
        self.messages_read(fail=False)
        if result == self.Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Gst PLAYING request failed; diagnostics='+json.dumps(self.messages))
        waited, current, pending = self.pipeline.get_state(min(timeout_ns, 2 * self.Gst.SECOND))
        self.messages_read()
        need(current == self.Gst.State.PLAYING, 'Gst did not reach PLAYING: '+str((waited, current, pending)))
        device_fd = self.source.get_property('device-fd')
        need(type(device_fd) is int and device_fd >= 0, 'Gst v4l2src exposes no opened device-fd')
        self.opened_device = signature(os.fstat(device_fd))
        need(self.opened_device == self.binding['device_identity'], 'Gst opened descriptor differs from pinned camera')

    def pull(self, timeout_ns):
        self.check()
        sample = self.sink.emit('try-pull-sample', timeout_ns)
        self.check()
        if sample is None:
            if self.sink.get_property('eos'):
                raise EOFError('Gst appsink reached EOS before the capture limit')
            raise TimeoutError('Gst appsink returned no sample before the bounded receipt timeout')
        caps = sample.get_caps()
        need(caps is not None and caps.get_size() == 1 and caps.is_fixed(), 'Gst sample caps are not fixed/single')
        structure = caps.get_structure(0)
        fraction = structure.get_fraction('framerate')
        need(fraction[0], 'Gst actual caps lack framerate')
        buffer = sample.get_buffer()
        need(buffer is not None, 'Gst sample lacks a buffer')
        size = buffer.get_size()
        need(0 < size <= MAX_FRAME_BYTES, 'Gst buffer size outside bounded JPEG budget: '+str(size))
        raw = buffer.extract_dup(0, size)
        need(len(raw) == size, 'Gst extract_dup length differs from buffer size')
        def timestamp(v):
            return None if v == self.Gst.CLOCK_TIME_NONE else int(v)
        def offset(v):
            return None if v == self.Gst.BUFFER_OFFSET_NONE else int(v)
        result = {'jpeg': bytes(raw), 'actual_caps': caps.to_string(), 'media_type': structure.get_name(),
            'width': structure.get_value('width'), 'height': structure.get_value('height'),
            'fps_num': fraction[1], 'fps_den': fraction[2], 'pts_ns': timestamp(buffer.pts),
            'dts_ns': timestamp(buffer.dts), 'duration_ns': timestamp(buffer.duration),
            'offset': offset(buffer.offset), 'offset_end': offset(buffer.offset_end),
            'buffer_flags': int(buffer.get_flags())}
        del buffer, sample
        return result

    def stop(self):
        try:
            self.messages_read(fail=False)
        except BaseException as error:
            self.error_seen = True
            self.messages.append({'type': 'DIAGNOSTIC_READ_FAILURE', 'message': str(error)[:4096]})
        result = self.pipeline.set_state(self.Gst.State.NULL)
        waited, current, pending = self.pipeline.get_state(0)
        self.null_confirmed = result != self.Gst.StateChangeReturn.FAILURE and current == self.Gst.State.NULL
        need(self.null_confirmed, 'Gst NULL cleanup unconfirmed: '+str((result, waited, current, pending)))
        return True

    def diagnostics(self):
        return {'messages': self.messages, 'messages_truncated': self.messages_truncated,
                'error_seen': self.error_seen, 'opened_device_identity': self.opened_device,
                'pipeline_null_confirmed': self.null_confirmed}


def read_boot():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def record(plan, output, *, backend_factory=None, binding_reader=device_binding,
           clock=time.monotonic_ns, boot_reader=read_boot, synthetic=False, ready_callback=None):
    need(plan == make_plan(plan['device'], plan['seconds'], plan['frames'], plan['max_bytes']), 'Noncanonical plan')
    need(plan['device'] is not None, 'Recording requires an explicit stable --device path')
    need(type(synthetic) is bool, 'Synthetic mode must be an explicit boolean')
    need(backend_factory is None or synthetic is True, 'Injected backends require explicit synthetic=True')
    need(not synthetic or backend_factory is not None, 'Synthetic mode requires an explicitly injected backend')
    backend_factory = GstCamera if backend_factory is None else backend_factory
    source_hash = regular_source()
    path, fd = private_output(output)
    report = {'schema': SCHEMA, 'status': 'INCOMPLETE', **FLAGS, 'plan': plan,
        'capture_backend': 'SYNTHETIC_TEST_INJECTION' if synthetic else 'LIVE_GST_V4L2_APPSINK',
        'synthetic_fixture': synthetic, 'camera_open_attempted': False, 'camera_opened': False,
        'boot_id': None, 'device_binding': None, 'source_sha256': source_hash,
        'frames': [], 'jpeg_bytes_saved': 0, 'errors': [], 'termination_reason': None,
        'pipeline_null_confirmed': False, 'gst_diagnostics': None,
        'exposure_s': None, 'timestamp_max_error_s': None, 'external_clock_alignment': None,
        'camera_frame_loss_count': None, 'physical_angle_resolution_rad': None,
        'timestamp_scope': 'Host monotonic receipt brackets only; Gst stream times retained without alignment.'}
    backend, previous = None, {}
    def interrupted(signum, _):
        raise InterruptedError('Camera recording interrupted by signal '+str(signum))
    try:
        write_member(fd, 'run.json', json_bytes({'plan': plan, 'source_sha256': report['source_sha256'],
                                               'capture_backend': report['capture_backend']}))
        if not synthetic:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, interrupted)
        report['boot_id'] = boot_reader()
        need(type(report['boot_id']) is str and bool(report['boot_id']), 'Boot identity missing')
        binding = binding_reader(plan['device']); report['device_binding'] = binding
        started = clock(); report['begin_monotonic_ns'] = started
        deadline = started + int(plan['seconds'] * 1e9)
        backend = backend_factory(plan, binding)
        report['startup_begin_monotonic_ns'] = clock()
        remaining = deadline-report['startup_begin_monotonic_ns']
        need(remaining > 0, 'Duration budget elapsed before camera startup')
        report['startup_get_state_wait_cap_ns'] = min(remaining, 2_000_000_000)
        report['camera_open_attempted'] = not synthetic
        report['camera_opened'] = None if not synthetic else False
        need(binding_reader(plan['device']) == binding, 'Camera binding changed before PLAYING')
        backend.start(remaining)
        report['startup_end_monotonic_ns'] = clock()
        need(backend.opened_device == binding['device_identity'], 'Opened camera descriptor binding differs')
        report['camera_opened'] = not synthetic
        while len(report['frames']) < plan['frames']:
            check_output(path, fd)
            need(binding_reader(plan['device']) == binding, 'Camera binding changed during recording')
            need(boot_reader() == report['boot_id'], 'Boot identity changed during recording')
            before = clock()
            if before >= deadline:
                report['termination_reason'] = 'DURATION_LIMIT'; break
            try:
                sample = backend.pull(min(2_000_000_000, deadline-before))
            except TimeoutError:
                after = clock()
                report['terminal_pull'] = {'host_pull_before_monotonic_ns': before,
                                          'host_pull_after_monotonic_ns': after, 'result': 'NO_SAMPLE_TIMEOUT'}
                backend.check()  # A duration endpoint must not conceal a Gst ERROR.
                if after >= deadline:
                    report['termination_reason'] = 'DURATION_LIMIT'; break
                raise
            after = clock()
            need(type(before) is int and type(after) is int and 0 <= before <= after, 'Invalid receipt clock bracket')
            metadata = validate_sample(sample); raw = sample['jpeg']
            if report['jpeg_bytes_saved'] + len(raw) > plan['max_bytes']:
                report['unretained_sample'] = {'size': len(raw), 'sha256': digest(raw), **metadata}
                report['termination_reason'] = 'BYTE_LIMIT'
                raise ValueError('Next JPEG exceeds total byte cap; partial prior frames retained')
            name = 'frame-%06d.jpg' % len(report['frames'])
            saved_hash = write_member(fd, name, raw)
            markers = jpeg_markers(raw)
            row = {'index': len(report['frames']), 'path': name, 'sha256': saved_hash, 'bytes': len(raw),
                   'host_pull_before_monotonic_ns': before, 'host_pull_after_monotonic_ns': after,
                   'exposure_s': None, 'timestamp_max_error_s': None, **metadata,
                   'jpeg_marker_observation': markers, 'receipt_after_deadline': after > deadline}
            report['frames'].append(row); report['jpeg_bytes_saved'] += len(raw)
            need(markers['starts_with_soi'] and markers['last_eoi_byte_offset'] is not None,
                 'Received payload lacks JPEG SOI/EOI markers; raw bytes retained')
            actual = {k: row[k] for k in ('media_type', 'width', 'height', 'fps_num', 'fps_den')}
            need(actual == plan['requested_caps'], 'Actual JPEG caps differ from requested mode; raw bytes retained')
            if len(report['frames']) == 1 and ready_callback is not None:
                ready_callback({'event': 'CAMERA_READY', 'boot_id': report['boot_id'],
                    'first_frame': row, 'source_sha256': report['source_sha256'],
                    'capture_backend': report['capture_backend'], **FLAGS})
        else:
            report['termination_reason'] = 'FRAME_LIMIT'
        backend.check()  # An ERROR cannot be masked by reaching the requested frame count.
        need(binding_reader(plan['device']) == binding, 'Camera binding changed after recording')
        need(bool(report['frames']), 'No camera frames acquired within the duration bound')
        need(regular_source() == report['source_sha256'], 'Recorder source changed during capture')
        report['status'] = ('SYNTHETIC_CAMERA_FIXTURE_REVIEW_REQUIRED' if synthetic else
                            'CAPTURED_CAMERA_FRAMES_REVIEW_REQUIRED')
    except BaseException as error:
        report['status'] = 'INTERRUPTED_PARTIAL' if isinstance(error, (KeyboardInterrupt, InterruptedError)) else 'INCOMPLETE'
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        if backend is not None:
            try:
                report['pipeline_null_confirmed'] = backend.stop() is True
                need(report['pipeline_null_confirmed'], 'Backend did not confirm NULL cleanup')
            except BaseException as error:
                report['status'] = 'INCOMPLETE'; report['errors'].append('Cleanup '+type(error).__name__+': '+str(error))
            try:
                report['gst_diagnostics'] = backend.diagnostics()
                diagnostics = report['gst_diagnostics']
                if diagnostics.get('error_seen') or any(m.get('type') == 'ERROR' for m in diagnostics.get('messages', [])):
                    report['status'] = 'INCOMPLETE'
                    report['errors'].append('Gst ERROR retained in final/cleanup diagnostics')
            except BaseException as error:
                report['status'] = 'INCOMPLETE'; report['errors'].append('Diagnostics '+type(error).__name__+': '+str(error))
        for sig, old in previous.items():
            signal.signal(sig, old)
        report['end_monotonic_ns'] = clock()
        try:
            try:
                check_output(path, fd)
                report['named_output_binding_valid'] = True
            except (OSError, ValueError) as error:
                report['status'] = 'INCOMPLETE'
                report['named_output_binding_valid'] = False
                report['errors'].append('Output binding: '+str(error))
            report['saved_frames_hash_verified'] = False
            try:
                for frame in report['frames']:
                    member = os.open(frame['path'], os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0), dir_fd=fd)
                    with os.fdopen(member, 'rb') as stream:
                        info = os.fstat(stream.fileno())
                        need(stat.S_ISREG(info.st_mode) and info.st_size == frame['bytes'], 'Saved frame type/size changed')
                        need(digest(stream.read(MAX_FRAME_BYTES+1)) == frame['sha256'], 'Saved frame hash changed')
                report['saved_frames_hash_verified'] = True
            except (OSError, ValueError) as error:
                report['status'] = 'INCOMPLETE'
                report['errors'].append('Saved frame verification: '+str(error))
            report_sha = write_member(fd, 'report.json', json_bytes(report))
        finally:
            os.close(fd)
    return report, {'report': str(path/'report.json'), 'report_sha256': report_sha}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-camera', action='store_true')
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--frames', type=int, default=300)
    parser.add_argument('--max-bytes', type=int, default=128*1024*1024)
    parser.add_argument('--output')
    args = parser.parse_args(argv)
    try:
        plan = make_plan(args.device, args.seconds, args.frames, args.max_bytes)
        if not args.record_camera:
            print(json.dumps(plan, ensure_ascii=False, indent=2, allow_nan=False)); return 0
        need(args.device is not None, '--record-camera requires an explicit stable --device path')
        need(args.output is not None, '--record-camera requires a fresh private --output directory')
        report, binding = record(plan, args.output,
            ready_callback=lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))
        print(json.dumps({'status': report['status'], 'frames_saved': len(report['frames']),
            'errors': report['errors'], 'pipeline_null_confirmed': report['pipeline_null_confirmed'],
            'gst_diagnostics': report['gst_diagnostics'], **FLAGS, **binding}, ensure_ascii=False, indent=2))
        return 0 if report['status'] == 'CAPTURED_CAMERA_FRAMES_REVIEW_REQUIRED' else 1
    except (OSError, ValueError) as error:
        print(json.dumps({'schema': SCHEMA, 'status': 'CAMERA_RECORDING_REFUSED', **FLAGS,
                          'error': type(error).__name__+': '+str(error)}, ensure_ascii=False)); return 1


if __name__ == '__main__':
    raise SystemExit(main())

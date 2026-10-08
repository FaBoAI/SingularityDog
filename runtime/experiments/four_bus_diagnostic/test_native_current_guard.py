"""Real read-only filesystem/native checks; no CAN, serial or model I/O."""
from contextlib import contextmanager, redirect_stdout
import ctypes as C
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import unittest
from unittest.mock import patch

from . import build_current_guard as build
from . import native_current_guard as ng
from . import topology


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def fixture(library, *, clock=time.monotonic_ns):
    """Only virtual null/zero/random nodes are fstat'ed; never read or write them."""
    with tempfile.TemporaryDirectory() as value:
        root = Path(value).resolve()
        ports, descriptors = {}, {}
        try:
            for port, device in zip(topology.PORTS, ('/dev/null', '/dev/zero', '/dev/random', '/dev/urandom')):
                alias = root/port
                resolved = str(Path(device).resolve(strict=True))
                alias.symlink_to(resolved)
                info = os.stat(resolved)
                ports[port] = {'path': str(alias), 'resolved': resolved, 'st_rdev': info.st_rdev}
                descriptors[port] = os.open(resolved, os.O_RDONLY | os.O_NONBLOCK)
            boot = root/'boot'
            boot.write_bytes(b'guard-test-boot\n')
            boot_fd = os.open(boot, os.O_RDONLY)
            stop = threading.Event()
            guard = ng.NativeCurrentGuard(library, ports, descriptors, boot_fd, 'guard-test-boot', stop, clock=clock)
            yield root, boot, boot_fd, ports, descriptors, stop, guard
        finally:
            if 'guard' in locals(): guard.close()
            if 'boot_fd' in locals():
                try: os.close(boot_fd)
                except OSError: pass
            for fd in descriptors.values(): os.close(fd)


class NativeCurrentGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name).resolve()
        cls.binary = build.build(cls.root/'build')
        cls.pins = dict(expected_sha256=digest(cls.binary),
            source_sha256=digest(cls.binary.with_name('native_current_guard.cpp')),
            build_record_sha256=digest(cls.binary.with_name('build-record.json')))
        cls.lib = ng.load_library(cls.binary, **cls.pins)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_abi_ctypes_sizes_offsets_and_original_gil_release_flag(self):
        self.assertEqual(self.lib.sdcg_abi(), 1)
        self.assertEqual(C.sizeof(ng.Pins), 46320)
        self.assertEqual(C.sizeof(ng.Timing), 56)
        self.assertEqual([ng.Pins.boot_identity.offset, ng.Pins.boot_bytes.offset,
            ng.Pins.parents.offset, ng.Pins.ports.offset], [16, 48, 176, 33712])
        self.assertEqual(self.lib.sdcg_check._flags_, C._FUNCFLAG_CDECL)

    def test_default_build_PLAN_creates_nothing(self):
        out = self.root/'absent-plan'
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(build.main(['--output', str(out)]), 0)
        self.assertFalse(out.exists())
        self.assertFalse(json.loads(output.getvalue())['opens_devices'])

    def test_genuine_success_exact_phase_order_and_pread_no_offset(self):
        with fixture(self.lib) as (_, _, fd, _, _, _, guard):
            os.lseek(fd, 3, os.SEEK_SET)
            guard()
            self.assertEqual(os.lseek(fd, 0, os.SEEK_CUR), 3)
            row = guard.calls[-1]
            self.assertTrue(row['ok'])
            native = row['native']
            stamps = [native[k] for k in ('started_ns','boot_checked_ns','ancestors_checked_ns','ports_checked_ns','finished_ns')]
            self.assertEqual(stamps, sorted(stamps))
            self.assertLessEqual(row['started_ns'], native['started_ns'])
            self.assertLessEqual(native['finished_ns'], row['finished_ns'])
            end = guard.finish()
            self.assertTrue(end['end_full_path_resolution_verified'])
            self.assertFalse(end['native_CAN_IO_available'])

    def test_all_five_concurrent_original_callers_have_private_results(self):
        with fixture(self.lib) as (_, _, _, _, _, _, guard):
            barrier = threading.Barrier(5)
            def check(): barrier.wait(); guard()
            with ThreadPoolExecutor(max_workers=5) as pool:
                futures = [pool.submit(check) for _ in range(5)]
                for future in futures: future.result()
            self.assertEqual(len(guard.calls), 5)
            self.assertEqual(len({row['thread_native_id'] for row in guard.calls}), 5)
            self.assertTrue(all(row['ok'] and row['native']['phase'] == 4 for row in guard.calls))

    def test_boot_bytes_change_truncated_and_whitespace_exact_python_strip(self):
        for contents, good in ((b'\t guard-test-boot \r\n',True),
                (b'wrong-boot\n',False),(b'guard-test-bo',False),(b'',False)):
            with self.subTest(contents=contents), fixture(self.lib) as (_,boot,_,_,_,_,guard):
                boot.write_bytes(contents)
                if good: guard()
                else:
                    with self.assertRaisesRegex(ValueError,'boot bytes changed'): guard()

    def test_closed_and_reused_boot_FD_rejected(self):
        with fixture(self.lib) as (root,_,fd,_,_,_,guard):
            os.close(fd)
            with self.assertRaisesRegex(ValueError,'boot fstat failed'): guard()
            other=root/'different';other.write_bytes(b'guard-test-boot\n')
            new=os.open(other,os.O_RDONLY)
            if new!=fd: os.dup2(new,fd);os.close(new)
            with self.assertRaisesRegex(ValueError,'boot FD identity changed'): guard()

    def test_parent_mode_and_identity_changes_rejected_before_ports(self):
        with fixture(self.lib) as (root,_,_,_,_,_,guard):
            mode = stat.S_IMODE(root.stat().st_mode)
            root.chmod(mode ^ 0o040)
            try:
                with self.assertRaisesRegex(ValueError,'ancestor directory identity changed'): guard()
                self.assertEqual(guard.calls[-1]['native']['phase'],2)
            finally: root.chmod(mode)

    def test_alias_recreation_is_rejected_even_same_target(self):
        with fixture(self.lib) as (_,_,_,ports,_,_,guard):
            p=Path(ports['port0']['path']);p.unlink();p.symlink_to(ports['port0']['resolved'])
            with self.assertRaisesRegex(ValueError,'alias identity/timestamps changed'):guard()

    def test_alias_metadata_change_is_rejected(self):
        with fixture(self.lib) as (_,_,_,ports,_,_,guard):
            p=Path(ports['port0']['path']);info=p.lstat()
            os.utime(p,ns=(info.st_atime_ns,info.st_mtime_ns+1_000_000),follow_symlinks=False)
            with self.assertRaisesRegex(ValueError,'alias identity/timestamps changed'):guard()

    def test_alias_retarget_cross_port_or_missing_rejected(self):
        for target in ('/dev/zero','/no-such-guard-test-device'):
            with self.subTest(target=target),fixture(self.lib) as (_,_,_,ports,_,_,guard):
                p=Path(ports['port0']['path']);p.unlink();p.symlink_to(target)
                with self.assertRaisesRegex(ValueError,'alias identity/timestamps changed'):guard()

    def test_CANCEL_before_native_and_finish_still_checks_after_cancel(self):
        with fixture(self.lib) as (_,_,_,_,_,stop,guard):
            stop.set()
            with self.assertRaisesRegex(ValueError,'Foreground cancelled'):guard()
            self.assertNotIn('native',guard.calls[-1])
            self.assertTrue(guard.finish()['calls'][-1]['ok'])

    def test_source_binary_and_build_pins_independently_reject(self):
        for key in self.pins:
            pins=dict(self.pins);pins[key]='0'*64
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'supplied source/build/binary SHA differs'):
                ng.load_library(self.binary,**pins)

    def test_foreign_CDLL_copied_or_cloned_proof_cannot_manufacture_registry(self):
        foreign=C.CDLL(str(self.binary))
        for proof in (self.lib._current_guard_proof,
                replace(self.lib._current_guard_proof,library=lambda:foreign,loaded_name=str(foreign._name))):
            foreign._current_guard_proof=proof
            with self.assertRaisesRegex(ValueError,'Genuine authenticated'):ng._verify_library(foreign)

    def test_selected_function_replacement_or_signature_mutation_rejected(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            original=self.lib.sdcg_check
            foreign=C.CDLL(str(self.binary)).sdcg_check
            foreign.argtypes=list(ng._ARGUMENTS);foreign.restype=C.c_int
            self.lib.sdcg_check=foreign
            try:
                with self.assertRaisesRegex(ValueError,'function identity/signature changed'):guard()
            finally:self.lib.sdcg_check=original
            original_args=original.argtypes
            original.argtypes=[]
            try:
                with self.assertRaisesRegex(ValueError,'function identity/signature changed'):guard()
            finally:original.argtypes=original_args

    def test_pin_or_original_setup_metadata_mutation_rejected_before_native(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            guard._pins.parent_count=2**32-1
            with self.assertRaisesRegex(ValueError,'ownership/pins changed'):guard()
            guard._pins.parent_count=len(guard._original.ancestors)
            saved=guard._original.boot_fd;guard._original.boot_fd=-1
            with self.assertRaisesRegex(ValueError,'ownership/pins changed'):guard()
            guard._original.boot_fd=saved

    def test_no_CDLL_abi_call_in_hot_path(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            # ABI was validated during loading; its function must remain sealed,
            # but no invocation is allowed during the hot dynamic check.
            with patch.object(ng, '_verify_library', wraps=ng._verify_library) as verify:
                guard()
            self.assertEqual(verify.call_count,2)
            self.assertEqual(guard.calls[-1]['native']['phase'],4)

    def test_final_clock_failure_always_settles_counter_and_preserves_native_primary(self):
        for native_failure in (False, True):
            count=0
            def clock():
                nonlocal count
                count+=1
                if count==2: raise RuntimeError('injected final clock failure')
                return time.monotonic_ns()
            with self.subTest(native_failure=native_failure),fixture(self.lib,clock=clock) as (_,boot,_,_,_,_,guard):
                if native_failure:boot.write_bytes(b'changed-boot\n')
                message='boot bytes changed' if native_failure else 'injected final clock failure'
                with self.assertRaisesRegex((ValueError,RuntimeError),message) as caught:guard()
                self.assertEqual(guard._active,0)
                self.assertEqual(len(guard.calls),1)
                self.assertIsNone(guard.calls[0]['finished_ns'])
                self.assertFalse(guard.calls[0]['ok'])
                if native_failure:self.assertIn('final clock',caught.exception.__notes__[0])

    def test_close_rejects_foreign_thread_and_closed_guard_reuse(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            with ThreadPoolExecutor(max_workers=1) as pool:
                with self.assertRaisesRegex(ValueError,'Original setup owner'):pool.submit(guard.close).result()
            guard.close()
            with self.assertRaisesRegex(ValueError,'ownership/pins changed'):guard()

    def test_direct_native_rejects_bounded_count_path_and_type_without_IO(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            for change in (lambda p:setattr(p,'parent_count',33),
                           lambda p:setattr(p.ports[0].target,'mode',stat.S_IFREG)):
                pins=ng.Pins.from_buffer_copy(bytes(guard._pins));change(pins)
                timing,error=ng.Timing(),C.create_string_buffer(512)
                status=self.lib.sdcg_check(C.byref(pins),C.byref(timing),error,len(error))
                self.assertEqual(status,-1);self.assertEqual(timing.phase,0)
                self.assertTrue(error.value)

    def test_every_parent_alias_link_follow_and_canonical_native_check_is_live(self):
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            mutations = (
                ('ancestor directory identity changed', lambda p:setattr(p.parents[0],'ino',p.parents[0].ino+1)),
                ('alias identity/timestamps changed', lambda p:setattr(p.ports[0].alias,'ctime_ns',p.ports[0].alias.ctime_ns+1)),
                ('alias link text changed', lambda p:setattr(p.ports[0],'link',b'/different-link-text')),
                ('alias target device identity changed', lambda p:setattr(p.ports[0].target,'rdev',p.ports[0].target.rdev+1)),
                ('canonical device identity changed', lambda p:setattr(p.ports[0],'resolved',os.fsencode('/dev/zero'))),
            )
            for message, change in mutations:
                with self.subTest(message=message):
                    pins=ng.Pins.from_buffer_copy(bytes(guard._pins));change(pins)
                    timing,error=ng.Timing(),C.create_string_buffer(512)
                    self.assertEqual(self.lib.sdcg_check(C.byref(pins),C.byref(timing),error,len(error)),-1)
                    self.assertIn(message,error.value.decode())

    def test_parent_replacement_and_symlink_directory_rejected(self):
        with fixture(self.lib) as (root,_,_,_,_,_,guard):
            renamed=root.with_name(root.name+'-retained')
            root.rename(renamed);root.mkdir()
            try:
                with self.assertRaisesRegex(ValueError,'ancestor directory identity changed'):guard()
            finally:root.rmdir();renamed.rename(root)
        with fixture(self.lib) as (_,_,_,_,_,_,guard):
            pins=ng.Pins.from_buffer_copy(bytes(guard._pins))
            pins.parents[0].path=pins.ports[0].path
            timing,error=ng.Timing(),C.create_string_buffer(512)
            self.assertEqual(self.lib.sdcg_check(C.byref(pins),C.byref(timing),error,len(error)),-1)
            self.assertIn(b'ancestor directory identity changed',error.value)

    def test_setup_rejects_regular_device_or_indirect_alias(self):
        with fixture(self.lib) as (root,_,fd,ports,descriptors,stop,_):
            regular=root/'not-a-character-device';regular.write_bytes(b'x')
            changed={k:dict(v) for k,v in ports.items()};p=Path(changed['port0']['path'])
            p.unlink();p.symlink_to(regular)
            with self.assertRaisesRegex(ValueError,'character device'):
                ng.NativeCurrentGuard(self.lib,changed,descriptors,fd,'guard-test-boot',stop)

    def test_partial_or_wrong_native_ABI_fails_before_guard_creation(self):
        source=build.SOURCE.read_text()
        # A separate source-authenticated binary with the selected ABI absent
        # must not silently fall back to Python or another loaded library.
        with tempfile.TemporaryDirectory() as value:
            root=Path(value).resolve();changed=root/'input.cpp'
            changed.write_text(source.replace('extern "C" uint32_t sdcg_abi()',
                'extern "C" uint32_t unsupported_sdcg_abi()').replace('!sdcg_abi()', '!unsupported_sdcg_abi()'))
            with patch.object(build,'SOURCE',changed):
                # Build helper intentionally writes the supplied filename;
                # retain the mandatory adjacent own-source locator for load.
                binary=build.build(root/'build')
            binary.with_name('native_current_guard.cpp').write_bytes(changed.read_bytes())
            with self.assertRaisesRegex(ValueError,'Complete genuine'):
                ng.load_library(binary,expected_sha256=digest(binary),source_sha256=digest(changed),
                    build_record_sha256=digest(binary.with_name('build-record.json')))

    def test_active_native_close_cancel_and_original_pin_mutation_fences(self):
        # ONLY this private test copy has a deterministic native pause hook.
        # The shipped source/build ABI contains no pause or test symbols.
        source=build.SOURCE.read_text()
        prefix='''#include <atomic>\n#include <thread>\nstatic std::atomic<int> test_entered{0},test_release{0};\nextern "C" int sdcg_test_entered(){return test_entered.load();}\nextern "C" void sdcg_test_release(int value){test_release.store(value);test_entered.store(0);}\n'''
        source=prefix+source.replace('timing->started_ns=now();',
            'timing->started_ns=now(); test_entered.store(1); while(!test_release.load()) std::this_thread::yield();',1)
        with tempfile.TemporaryDirectory() as value:
            root=Path(value).resolve();changed=root/'native_current_guard.cpp';changed.write_text(source)
            with patch.object(build,'SOURCE',changed):binary=build.build(root/'build')
            library=ng.load_library(binary,expected_sha256=digest(binary),source_sha256=digest(changed),
                build_record_sha256=digest(binary.with_name('build-record.json')))
            for which in ('cancel','pins','native_failure_and_cancel'):
                with self.subTest(which=which),fixture(library) as (_,boot,_,_,_,stop,guard), \
                        ThreadPoolExecutor(max_workers=1) as pool:
                    library.sdcg_test_release(0)
                    future=pool.submit(guard)
                    limit=time.monotonic()+2
                    while not library.sdcg_test_entered() and time.monotonic()<limit:time.sleep(.001)
                    try:
                        self.assertTrue(library.sdcg_test_entered())
                        with self.assertRaisesRegex(ValueError,'Join original'):guard.close()
                        if which=='cancel':stop.set()
                        elif which=='native_failure_and_cancel':boot.write_bytes(b'changed-boot\n');stop.set()
                        else:guard._pins.parent_count=2**32-1
                    finally:library.sdcg_test_release(1)
                    message=('Foreground cancelled' if which=='cancel' else
                             'boot bytes changed' if which=='native_failure_and_cancel' else 'ownership/pins changed')
                    try:
                        with self.assertRaisesRegex(ValueError,message):future.result(timeout=2)
                        # The actual C call used its private canonical snapshot
                        # and completed all checks without OOB/rebound boot read.
                        self.assertEqual(guard.calls[-1]['native']['phase'],1 if which=='native_failure_and_cancel' else 4)
                        self.assertTrue(guard.calls[-1]['cancellation_after_native_checked'])
                        self.assertFalse(guard.calls[-1]['ok'])
                    finally:guard._pins.parent_count=len(guard._original.ancestors)


if __name__=='__main__':unittest.main()

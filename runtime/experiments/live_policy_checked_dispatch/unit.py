"""Explicit CPU guard/state fixture entry. Default PLAN imports no Torch."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest

if __package__:
    from .file_inputs import need, pinned, verify_pins
else:
    from file_inputs import need, pinned, verify_pins

HERE = Path(__file__).absolute().parent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--runtime-root', required=True, type=Path)
    parser.add_argument('--original-source-sha256', required=True)
    parser.add_argument('--build-record', required=True, type=Path)
    parser.add_argument('--build-record-sha256', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    need(not sys.flags.optimize and sys.dont_write_bytecode, 'Unsuppressed python -B required')
    runtime = args.runtime_root.absolute()
    record_path = args.build_record.absolute()
    record = json.loads(pinned(record_path, args.build_record_sha256))
    source = runtime / 'singularitydog_hw/policy_output_model.py'
    pinned(source, args.original_source_sha256)
    cpp = HERE / 'checked_dispatch.cpp'
    pinned(cpp, record['source_sha256'])
    library = record_path.parent / 'live_checked_dispatch.so'
    pinned(library, record['library_sha256'])
    pins = {str(source): args.original_source_sha256, str(record_path): args.build_record_sha256,
            str(cpp): record['source_sha256'], str(library): record['library_sha256']}
    for name in ('unit.py', 'file_inputs.py', 'adapter.py', 'cpu_kernel.py',
                 'test_component.py', 'test_kernel.py'):
        path = HERE / name
        pins[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    plan = {'schema': 'experimental.live-checked-dispatch-unit.v1', 'status': 'PLAN_ONLY',
            'input_sha256': pins, 'hardware_opened': False, 'output_allowed': False,
            'approved_for_runtime': False, 'actual_runtime_qualification': False,
            'fixture_sensor_validation_is_injected': True}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    original_path, original_env = sys.path[:], os.environ.copy()
    try:
        sys.path[:0] = [str(HERE), str(runtime)]
        os.environ.update(LIVE_COMPONENT_RUNTIME=str(runtime),
                          LIVE_COMPONENT_ORIGINAL_SOURCE_SHA256=args.original_source_sha256,
                          LIVE_COMPONENT_BUILD=str(record_path.parent),
                          LIVE_COMPONENT_BUILD_SHA256=args.build_record_sha256)
        import test_component
        import test_kernel
        suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(module)
                                   for module in (test_component, test_kernel))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        verify_pins(pins)
        need(result.wasSuccessful() and result.testsRun == 15 and not result.skipped,
             'All fifteen CPU guard/state fixtures must pass')
    finally:
        sys.path[:] = original_path
        os.environ.clear()
        os.environ.update(original_env)
    print(json.dumps({**plan, 'status': 'PASS_15_CPU_FIXTURES_NO_RUNTIME_QUALIFICATION',
                      'tests_run': result.testsRun, 'input_pins_unchanged': True}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

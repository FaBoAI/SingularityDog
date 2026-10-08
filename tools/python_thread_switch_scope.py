"""Run one robot diagnostic/output module with a reversible Python switch interval.

This changes Python thread scheduling only. The selected module still validates
its own hardware authorization, limits, source pins, and finite run duration.
It does not change CAN pacing, deadlines, or actuator gains.
"""

import argparse
import json
import math
import runpy
import sys


MODULES = (
    'singularitydog_hw.native_pipeline_benchmark',
    'singularitydog_hw.policy_output',
)


def run(module, arguments, *, interval_us, runner=runpy.run_module, emit=print):
    if module not in MODULES or type(interval_us) is not int or interval_us != 100:
        raise ValueError('Only the reviewed 100us switch comparison is supported')
    before, argv = sys.getswitchinterval(), sys.argv
    primary = None
    try:
        sys.setswitchinterval(interval_us / 1_000_000)
        emit(json.dumps({'kind': 'python_switch_interval', 'module': module,
                         'before_s': before, 'during_s': sys.getswitchinterval()}))
        sys.argv = [module, *arguments]
        return runner(module, run_name='__main__')
    except BaseException as error:
        primary = error
        raise
    finally:
        sys.argv = argv
        restoration_error = None
        try:
            # CPython quantizes the setter to integer microseconds. Its getter
            # can return 100us as 99.99999999999999us; setting that float back
            # otherwise truncates a nested scope to 99us. Advance one float
            # step for the setter, then require the exact original readback.
            sys.setswitchinterval(math.nextafter(before, math.inf))
            if sys.getswitchinterval() != before:
                raise RuntimeError('Original Python switch interval was not restored')
        except BaseException as error:
            restoration_error = error
        emit(json.dumps({'kind': 'python_switch_interval_restore',
                         'after_s': sys.getswitchinterval(),
                         'restored': restoration_error is None and sys.getswitchinterval() == before}))
        if restoration_error is not None:
            if primary is not None:
                primary.add_note('Python switch interval restoration failed: ' + repr(restoration_error))
            else:
                raise RuntimeError('Python switch interval restoration failed') from restoration_error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module', choices=MODULES, required=True)
    parser.add_argument('--interval-us', type=int, choices=(100,), required=True)
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    arguments = args.arguments
    if arguments[:1] == ['--']:
        arguments = arguments[1:]
    run(args.module, arguments, interval_us=args.interval_us)


if __name__ == '__main__':
    main()

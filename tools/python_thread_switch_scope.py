"""Run one robot diagnostic/output module with a reversible Python switch interval.

This changes Python thread scheduling only. The selected module still validates
its own hardware authorization, limits, source pins, and finite run duration.
It does not change CAN pacing, deadlines, or actuator gains.
"""

import argparse
import json
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
    try:
        sys.setswitchinterval(interval_us / 1_000_000)
        emit(json.dumps({'kind': 'python_switch_interval', 'module': module,
                         'before_s': before, 'during_s': sys.getswitchinterval()}))
        sys.argv = [module, *arguments]
        return runner(module, run_name='__main__')
    finally:
        sys.argv = argv
        sys.setswitchinterval(before)
        emit(json.dumps({'kind': 'python_switch_interval_restore',
                         'after_s': sys.getswitchinterval(),
                         'restored': sys.getswitchinterval() == before}))


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

"""Print a file-only instrumentation proposal. No execution/build mode exists."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat

EXPECTED_SOURCE = '1f8aebaccf3cdbea84d1e8945cd4510b9fc27ce13ef998f3129b8ab6a80d8646'
DEFAULT_SOURCE = Path(__file__).resolve().parent.parent/'native_transport/transport.cpp'


def plan(source=DEFAULT_SOURCE):
    path = Path(source)
    if not path.is_absolute() or '..' in path.parts or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('Absolute regular source without symlinks required')
    fd = os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size>65536:
            raise ValueError('Bounded regular source required')
        raw=stream.read(65537)
    if len(raw)>65536 or hashlib.sha256(raw).hexdigest()!=EXPECTED_SOURCE:
        raise ValueError('Exact corrected diagnostic source required')
    return {'schema':'singularitydog.native-causal-trace-plan.v1','status':'PLAN_ONLY',
        'source_sha256':EXPECTED_SOURCE,'candidate_mode':'separate diagnostic STOP-proxy library only',
        'existing_transport_abi_changed':False,'instrumentation_integrated':False,
        'hardware_opened':False,'library_loaded':False,'compiled':False,
        'output_allowed':False,'timing_admission_eligible':False,
        'proposed_slots_per_bus_owner':3,'proposed_events_per_slot':256,
        'slot_names':['acquisition','voltage','output'],
        'required_binding':['owner thread','bus','native begin_ns','request IDs','absolute deadline'],
        'required_future_events':['boot pread bracket','pselect bracket/timeout/ready bits','read chunk size with original timestamps','write with original timestamps'],
        'primitive_available':'Fixed POD recorder and fake-call errno/overflow regression only',
        'still_required':['Explicit separately pinned trace-only entry point/adapter','owner and slot binding','ready-bit capture','failed-coordinator/native-success retention after joins','serialization and end-hash validation','measured instrumentation overhead'],
        'limitations':['Failure-only persistence still instruments measured calls.','Optional CPU clocks increase overhead.','Host ready/read events cannot prove CAN/USB wire arrival.','No causal finding or active timing qualification follows from this plan.']}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--source',type=Path,default=DEFAULT_SOURCE)
    args=parser.parse_args(argv)
    print(json.dumps(plan(args.source),sort_keys=True,allow_nan=False))
    return 0


if __name__=='__main__':raise SystemExit(main())

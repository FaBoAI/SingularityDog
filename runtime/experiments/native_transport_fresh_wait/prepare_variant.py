"""Stage exact legacy or corrected diagnostic source; never build or run it."""
from pathlib import Path
import argparse
import difflib
import hashlib
import json

SOURCE_SHA256 = '5461c925575bd10bbca2a0c8c8d2c28cd119f22228bf7f8d7ed3976dcd73da0e'
CORRECTED_SHA256 = '1f8aebaccf3cdbea84d1e8945cd4510b9fc27ce13ef998f3129b8ab6a80d8646'
BEFORE = '''        uint64_t wait=wake>t?wake-t:0;
        timespec timeout{time_t(wait/1000000000),long(wait%1000000000)};
'''
AFTER = '''        // Boot checking may consume time after loop-entry t.
        // Keep the original absolute wake/deadline; never extend either one.
        const uint64_t before_wait=now();
        if(!before_wait||before_wait>=deadline) return deadline_fail();
        const uint64_t wait=wake>before_wait?wake-before_wait:0;
        timespec timeout{time_t(wait/1000000000),long(wait%1000000000)};
'''

def sha(data):
    return hashlib.sha256(data).hexdigest()


def source_version(source):
    if type(source) is not bytes:
        raise ValueError('Exact reviewed diagnostic source bytes required')
    digest = sha(source)
    if digest == SOURCE_SHA256:
        return 'legacy_stale_relative_wait'
    if digest == CORRECTED_SHA256:
        return 'corrected_fresh_relative_wait'
    raise ValueError('Exact reviewed diagnostic source bytes required')


def derive(source):
    version = source_version(source)
    if version == 'corrected_fresh_relative_wait':
        return source
    text = source.decode('utf-8')
    if text.count(BEFORE) != 1:
        raise ValueError('Unique original wait computation required')
    result = text.replace(BEFORE, AFTER).encode('utf-8')
    if result.decode().replace(AFTER, BEFORE) != text:
        raise ValueError('Unexpected source delta')
    if sha(result) != CORRECTED_SHA256:
        raise ValueError('Corrected source bytes differ from reviewed version')
    return result


def prepare(source_path, output=None):
    source_path = Path(source_path).resolve(strict=True)
    source = source_path.read_bytes()
    version = source_version(source)
    derived = derive(source)
    delta = ''.join(difflib.unified_diff(source.decode().splitlines(True),
        derived.decode().splitlines(True), fromfile='original/transport.cpp',
        tofile='corrected/transport.cpp'))
    result = {'schema': 'singularitydog.diagnostic-fresh-wait-variant.v2',
        'status': 'PLAN_ONLY' if output is None else 'SOURCE_VARIANT_PREPARED',
        'input_source_version': version,
        'output_source_version': 'corrected_fresh_relative_wait',
        'transformation': 'legacy_block_replaced' if sha(source) != sha(derived) else 'already_corrected_no_delta',
        'source_sha256': sha(source), 'variant_sha256': sha(derived),
        'diff_sha256': sha(delta.encode()), 'source_path': str(source_path),
        'change': 'Recompute relative pselect wait from unchanged absolute wake after boot read; abort if original deadline expired.',
        'hardware_opened': False, 'native_library_loaded': False,
        'compiled': False, 'production_changed': False,
        'source_only': True, 'timing_admission_eligible': False,
        'approved_for_runtime': False, 'output_allowed': False,
        'limitations': ['Adds one monotonic clock read per loop, including ready/past-wake polls.',
            'No real USB/CAN latency or target performance claim.',
            'New source/binary needs independent build and exact-source disabled measurement before any use.',
            'Original absolute deadline, cancellation, framing, matching and no-retry behavior are retained.']}
    if output is not None:
        target = Path(output)
        if not target.is_absolute() or target.exists():
            raise ValueError('Fresh absolute output directory required')
        target.mkdir(mode=0o700, parents=False)
        try:
            (target/'original-transport.cpp').write_bytes(source)
            (target/'transport.cpp').write_bytes(derived)
            (target/'source.diff').write_text(delta)
            if sha(source_path.read_bytes()) != sha(source):
                raise ValueError('Source changed during preparation')
            result['files'] = {p.name: sha(p.read_bytes()) for p in target.iterdir()}
            (target/'variant-manifest.json').write_text(json.dumps(result, indent=2)+'\n')
        except BaseException:
            # Incomplete source staging is retained, with no complete manifest.
            (target/'variant-manifest.json').unlink(missing_ok=True)
            raise
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', type=Path, help='Fresh directory; omit for file-only PLAN')
    args = parser.parse_args(argv)
    print(json.dumps(prepare(args.source, args.output), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

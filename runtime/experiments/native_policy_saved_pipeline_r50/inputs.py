"""Pinned historical file closure and exactly 33 completed archived snapshots."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import struct
from types import ModuleType

SCHEMA = 'singularitydog.saved-pipeline-source-bundle.r50.v1'
NAMES = frozenset(('__init__.py', 'adapter.py', 'inputs.py', 'support.py', 'package.py',
                  'test_adapter.py', 'test_inputs.py', 'test_package.py', 'README.md'))
PINS = {
    'report': '4369859814b0d864c67e42714187ce1b527771b70eeeb94cc7abd6ed0e5c6260',
    'records': 'cf1e75e54f119d78db5afdf463c37ac9fce78eba7da35a2398b36ca625f15d71',
    'calibration': '21eedf31f4d548f663e19669e48edc60f5f84ad355e32d49631a96e4ebfd7c0e',
    'mount': 'b9f6814e13141e82efca0fbb64772847ae7ad91434282b8c9b1e038b3c42b420',
    'gyro_bias': '2ade7e77705f51f077386154bbc70180f3cec614cc5ecaea2c439dac964cb8f8',
    'accel_hypothesis': '6cbc3426b5bd0b1f4daf090129704c5584af5b51efac661a4f6e83d8e4975415',
    'r48_source_manifest': 'df8f43f3d7627afb14e481569ce41a607e54780674018010f6d0db5defa729f4',
    'r47_source_manifest': '1e08148e7426172f6123c699e8ce52f10663adb2ab6adaf08a6623048aca9a17',
    'fk_candidate_original': 'd68c138ba5345010fd9020b3e49d89ffbccec6fed3515c30d19360b42c695220',
    'fk_candidate_rebound': '23b40c517b4351cd4c52458e142eb4580546379775afa5c5ced1d0e6db8a6be6',
    'r49_source_manifest': 'd665a415aa646662a339b98a00910f06e400ee0bbf4cb1f0b4bf0072aeeef6cc',
    'snapshot_artifact': '41552932cb23ef163c56df57f8abaa3ba9febd572bc869cc5b9f5f2cdad164ec',
}
KIT_PIN = 'f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'
MODEL_PIN = '1085a71881e8366633d5ac063c5e3de54eca335be5faab204b8cf867e22e11bb'
CAL_CANONICAL = 'd2cfbd67797933f7fa83bda0c1de25453e1040f80ff0fe8a6bdd6300b72b4c71'
FALSE_FLAGS = ('hardware_opened', 'output_allowed', 'approved_for_runtime',
    'active_controller_qualification', 'timing_admission_eligible',
    'live_50hz_verified', 'whole_20ms_gain_proven', 'ordinary_runtime_changed')
WIDTHS = {'gyro_body_rad_s': 3, 'gravity_body_unit': 3, 'command': 3,
          'q_model_rad': 12, 'dq_model_rad_s': 12, 'h_hypothesis12': 12}
OUTPUTS = {'q_target_rad_diagnostic_only': 12, 'actor_residual12': 12, 'observation74': 74}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def floating(value):
        result = float(value)
        require(math.isfinite(result), 'Nonfinite JSON number')
        return result
    def invalid(value):
        raise ValueError('Nonfinite JSON constant')
    return json.loads(raw, object_pairs_hook=pairs, parse_float=floating, parse_constant=invalid)


def path(value):
    require(type(value) is str and value.startswith('/') and '\\' not in value, 'Absolute path required')
    result = Path(value)
    require(str(result) == value and '..' not in result.parts, 'Canonical path required')
    require(not any(p.is_symlink() for p in (result, *result.parents)), 'Symlink path rejected')
    return result


def reference(value):
    require(type(value) is dict and set(value) == {'path', 'sha256'}, 'Exact path/SHA256 reference required')
    require(type(value['sha256']) is str and re.fullmatch('[0-9a-f]{64}', value['sha256']), 'SHA256 required')
    return path(value['path'])


def read(ref):
    filename = reference(ref)
    fd = os.open(filename, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and 0 <= before.st_size <= 128 * 1024 * 1024,
                'Bounded regular file required')
        with os.fdopen(fd, 'rb', closefd=False) as handle:
            raw = handle.read(before.st_size + 1)
        after = os.fstat(fd)
        named = filename.lstat()
        identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns)
        require(identity(before) == identity(after) == identity(named) and
                len(raw) == before.st_size and sha(raw) == ref['sha256'], 'Pinned file changed: ' + str(filename))
        path(str(filename))
        return raw
    finally:
        os.close(fd)


def relative(value):
    require(type(value) is str and value and '\\' not in value, 'Relative member required')
    item = Path(value)
    require(not item.is_absolute() and str(item) == value and '..' not in item.parts,
            'Canonical relative member required')
    return value


class Closure:
    """Read only exact target references; no path remapping in execution."""
    def __init__(self):
        self.pins = {}
    def read(self, ref):
        reference(ref)
        require(ref['path'] not in self.pins or self.pins[ref['path']] == ref['sha256'], 'Conflicting file pin')
        raw = read(ref)
        self.pins[ref['path']] = ref['sha256']
        return raw
    def verify(self):
        for filename, pin in tuple(self.pins.items()):
            read({'path': filename, 'sha256': pin})
    def nested(self, value):
        """Recursively authenticate explicit refs, including referenced JSON refs."""
        if type(value) is dict:
            if set(value) == {'path', 'sha256'}:
                already = value['path'] in self.pins
                raw = self.read(value)
                if not already and Path(value['path']).suffix == '.json':
                    self.nested(strict_json(raw))
            else:
                for item in value.values():
                    self.nested(item)
        elif type(value) is list:
            for item in value:
                self.nested(item)


def verify_bundle(manifest, expected_sha256, *, executing_adapter=None):
    closure = Closure()
    document = strict_json(closure.read({'path': str(manifest), 'sha256': expected_sha256}))
    require(type(document) is dict and document.get('schema') == SCHEMA and
            document.get('mode') == 'HISTORICAL_SAVED_PIPELINE_ONLY' and
            all(document.get(flag) is False for flag in FALSE_FLAGS), 'Exact saved-only manifest scope required')
    require(set(document) == {'schema', 'source_files', 'references', 'target_bundle_path', 'mode', *FALSE_FLAGS},
            'Unknown manifest fields')
    folder = path(str(manifest)).parent
    require(document['target_bundle_path'] == str(folder), 'Executing bundle path differs')
    sources = document['source_files']
    require(type(sources) is dict and set(sources) == NAMES, 'Exact nine source inventory required')
    require({str(p.relative_to(folder)) for p in folder.rglob('*')} == NAMES | {'manifest.json'},
            'Extra source member rejected')
    for name, pin in sources.items():
        closure.read({'path': str(folder / name), 'sha256': pin})
    if executing_adapter is not None:
        require(path(str(executing_adapter)) == folder / 'adapter.py', 'Executing adapter path differs')
    refs = document['references']
    require(type(refs) is dict and set(refs) == set(PINS), 'Exact twelve historical references required')
    docs = {}
    for role, ref in refs.items():
        require(ref['sha256'] == PINS[role], 'Historical reference differs: ' + role)
        docs[role] = strict_json(closure.read(ref))
    inventories = {}
    for role, key, count in (('r48_source_manifest', 'source_sha256', 21),
                            ('r47_source_manifest', 'files', 13), ('r49_source_manifest', 'files', 23)):
        inventory = docs[role][key]
        require(type(inventory) is dict and len(inventory) == count, 'Frozen source inventory differs')
        root = reference(refs[role]).parent
        if 'target_bundle_path' in docs[role]:
            require(docs[role]['target_bundle_path'] == str(root), 'Frozen source location differs')
        for name, pin in inventory.items():
            closure.read({'path': str(root / relative(name)), 'sha256': pin})
        inventories[role] = inventory
    r49 = docs['r49_source_manifest']
    require(r49['baseline_kit_manifest_sha256'] == KIT_PIN, 'Exact K37 kit required')
    kit = path(r49['baseline_kit_path'])
    kit_doc = strict_json(closure.read({'path': str(kit / 'kit-manifest.json'), 'sha256': KIT_PIN}))
    require(type(kit_doc['files']) is dict and len(kit_doc['files']) == 650, 'Exact K37 650 source files required')
    for name, pin in kit_doc['files'].items():
        closure.read({'path': str(kit / relative(name)), 'sha256': pin})
    old = docs['fk_candidate_original']; new = docs['fk_candidate_rebound']
    require({k: v for k, v in old.items() if k != 'integration_sources'} ==
            {k: v for k, v in new.items() if k != 'integration_sources'}, 'Only FK five integration refs may differ')
    expected_names = {'diagnostic_loader.py', 'diagnostic_support.py', 'diagnostic_generate.py',
                      'diagnostic_child.py', 'fk_cache_diagnostic_benchmark.py'}
    require(set(old['integration_sources']) == set(new['integration_sources']) == expected_names,
            'Exact FK five integration refs required')
    r49root = reference(refs['r49_source_manifest']).parent
    for name, ref in new['integration_sources'].items():
        member = ('native_policy_overnight/target_tail_fk_cache/' + name if name == 'diagnostic_loader.py' else name)
        require(ref == {'path': str(r49root / member), 'sha256': inventories['r49_source_manifest'][member]},
                'Rebound FK integration source differs')
    require(old['references']['model']['sha256'] == MODEL_PIN, 'Exact FK model required')
    require(docs['snapshot_artifact']['references']['source_bundle'] == refs['r49_source_manifest'],
            'Exact R49 snapshot source binding required')
    for role in ('fk_candidate_original', 'fk_candidate_rebound', 'snapshot_artifact',
                 'calibration', 'gyro_bias', 'accel_hypothesis'):
        closure.nested(docs[role])
    closure.verify()
    return document, docs, inventories, kit_doc, closure


def float32_bits(values, width):
    require(type(values) is list and len(values) == width, 'Vector width differs')
    result = []
    for value in values:
        require(type(value) in (int, float) and math.isfinite(value), 'Finite numeric vector required')
        try:
            raw = struct.pack('=f', value)
        except (OverflowError, struct.error):
            raise ValueError('Float32 overflow') from None
        require(math.isfinite(struct.unpack('=f', raw)[0]), 'Float32 overflow')
        result.append(raw)
    return b''.join(result)


def archived_rows(report, records, calibration, refs):
    require(report.get('status') == 'ABORTED' and type(report.get('cycles_completed')) is int
            and report['cycles_completed'] == 33, 'Exact aborted R37 completed33 report required')
    require(all(report.get(name) is False for name in ('motor_enable_sent', 'learned_targets_sent',
            'approved_for_runtime', 'full_controller_50Hz_verified')), 'Original archived no-output scope required')
    require(type(records) is list and len(records) == 34, 'Original 34 records including failed partial required')
    require(all(report['input_sha256'][key] == refs[role]['sha256'] for key, role in
            (('calibration', 'calibration'), ('mount', 'mount'), ('gyro_bias', 'gyro_bias'),
             ('accel_input_hypothesis', 'accel_hypothesis'))), 'Historical calibration/input selection differs')
    require(sha(canonical(calibration)) == CAL_CANONICAL, 'Historical canonical calibration differs')
    complete = records[:33]
    for index, row in enumerate(complete):
        require(type(row) is dict and type(row.get('cycle')) is int and row['cycle'] == index + 1,
                'Original cycle order differs')
        observed = row.get('observed')
        require(type(observed) is dict and observed.get('status') == 'TICK_OBSERVED_NO_OUTPUT' and
                observed.get('output_allowed') is False and type(observed.get('tick_index')) is int and
                observed['tick_index'] == index and type(observed.get('tick_ns')) is int and
                observed['tick_ns'] > 0, 'Completed original tick required')
        require(set(observed['inputs']) == set(WIDTHS), 'Exact six inputs required')
        for name, width in WIDTHS.items():
            float32_bits(observed['inputs'][name], width)
        for name, width in OUTPUTS.items():
            float32_bits(observed[name], width)
        require(observed['inputs']['command'] == [0., 0., 0.] and
                observed['inputs']['h_hypothesis12'] == [0.] * 12, 'Archived STOP/h0 inputs required')
        require(observed['provenance']['calibration_canonical_json_sha256'] == CAL_CANONICAL,
                'Per-tick archived calibration digest differs')
    partial = records[33]
    require(type(partial) is dict and partial.get('cycle') == 34 and not partial.get('observed'),
            'Failed partial34 must remain excluded')
    ticks = [row['observed']['tick_ns'] for row in complete]
    require(all(a < b for a, b in zip(ticks, ticks[1:])), 'Original monotonic ticks required')
    return complete, partial


def snapshots(document, docs, inventories, closure):
    refs = document['references']
    rows, partial = archived_rows(docs['report'], docs['records'], docs['calibration'], refs)
    root = reference(refs['r48_source_manifest']).parent
    harness_path = root / 'harness.py'
    module = ModuleType('_r50_archived_pure_decoder_harness')
    module.__file__ = str(harness_path)
    exec(compile(closure.read({'path': str(harness_path), 'sha256': inventories['r48_source_manifest']['harness.py']}),
                 str(harness_path), 'exec'), module.__dict__)
    _, module.PINS = module.verify_manifest(reference(refs['r48_source_manifest']),
                                           refs['r48_source_manifest']['sha256'], reference(refs['records']))
    module.RECORDS = reference(refs['records'])
    engine = module.load_pure_decoder()
    saved, digests = module.saved_snapshots(engine)
    require(len(rows) == len(saved) == len(digests) == 33, 'Exact reconstructed33 required')
    for row, snapshot, digest in zip(rows, saved, digests):
        require(snapshot['tick_ns'] == row['observed']['tick_ns'] and
                digest == row['observed']['provenance']['snapshot_canonical_json_sha256'], 'Dynamic snapshot digest differs')
    for filename, pin in module.PINS.items():
        closure.read({'path': filename, 'sha256': pin})
    for filename, pin in engine.reconstruction_source_sha256.items():
        closure.read({'path': str(filename), 'sha256': pin})
    closure.verify()
    return rows, saved, digests, partial

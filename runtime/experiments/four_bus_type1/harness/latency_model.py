"""Empirical USB2CAN/RS05 reply-latency model for the motor-free timing harness.

``extract`` reads recorded four-bus reports (Type1 zero-gain runs and STOP-proxy
diagnostics) and keeps only per-exchange reply delays, relative to the host
write timestamps the native transport recorded:

* ``burst_first``: first reply of a three-request burst, ``received[0]-start[0]``
* ``burst_rest``: replies two and three, ``received[k]-start[2]`` (they arrive
  batched after the last write of the burst)
* ``single``: the one-request Type17 voltage read, ``received[0]-start[0]``

Each category is kept separately for Type1 bursts (mode-2 replies) and STOP
bursts (mode-0 replies). The output JSON is small and is committed next to this
file so the Jetson run needs no 50 MB report. This module opens no device.
"""
import argparse
import json
import math
from pathlib import Path
import random

SCHEMA = 'singularitydog.four-bus-type1-harness-latency-model.v1'
DEFAULT_PATH = Path(__file__).with_name('latency_model.json')


def _batches(report):
    """Yield (kind, records) for every recorded per-port hold/voltage/output batch."""
    measurement = report.get('measurement') or {}
    rows = measurement.get('cycles')
    if not isinstance(rows, list):
        rows = measurement.get('records') or []
    for row in rows:
        for stage in ('hold', 'feedback', 'voltage', 'output'):
            value = row.get(stage)
            if not isinstance(value, dict):
                continue
            for batch in value.values():
                records = batch.get('records') if isinstance(batch, dict) else None
                if not records or any(r.get('received') != 17 or r.get('written') != 17 for r in records):
                    continue
                kind = int(records[0]['tx_hex'][4:12], 16) >> 27
                yield kind, records


def extract(paths):
    model = {'schema': SCHEMA, 'sources': [], 'units': 'ns',
             'type1': {'burst_first': [], 'burst_rest': [], 'rest_spread': []},
             'stop': {'burst_first': [], 'burst_rest': [], 'rest_spread': []},
             'single': [], 'write_gap': []}
    for path in paths:
        report = json.loads(Path(path).read_text())
        count = 0
        for kind, records in _batches(report):
            if len(records) == 3 and kind in (1, 4):
                bucket = model['type1' if kind == 1 else 'stop']
                bucket['burst_first'].append(records[0]['received_ns']-records[0]['start_ns'])
                last = records[2]['start_ns']
                bucket['burst_rest'].append(records[1]['received_ns']-last)
                bucket['rest_spread'].append(records[2]['received_ns']-records[1]['received_ns'])
                model['write_gap'] += [records[1]['start_ns']-records[0]['start_ns'],
                                       records[2]['start_ns']-records[1]['start_ns']]
                count += 1
            elif len(records) == 1 and kind == 17:
                model['single'].append(records[0]['received_ns']-records[0]['start_ns'])
                count += 1
        model['sources'].append({'name': Path(path).name, 'batches': count,
                                 'status': report.get('status'),
                                 'schema': report.get('schema')})
    model['summary'] = {}
    for name, values in (('type1.burst_first', model['type1']['burst_first']),
                         ('type1.burst_rest', model['type1']['burst_rest']),
                         ('stop.burst_first', model['stop']['burst_first']),
                         ('stop.burst_rest', model['stop']['burst_rest']),
                         ('single', model['single']), ('write_gap', model['write_gap'])):
        model['summary'][name] = describe(values)
    return model


def quantile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values)-1)*fraction
    low = int(math.floor(position)); high = min(low+1, len(values)-1)
    return values[low]+(values[high]-values[low])*(position-low)


def describe(values, scale=1e-6):
    if not values:
        return {'n': 0}
    return {'n': len(values), 'min': min(values)*scale, 'median': quantile(values, .5)*scale,
            'p90': quantile(values, .9)*scale, 'p99': quantile(values, .99)*scale,
            'p999': quantile(values, .999)*scale, 'max': max(values)*scale}


class Sampler:
    """Reply-delay sampler: ``empirical`` bootstrap, ``median`` (fixed) or ``fixed`` explicit values."""
    def __init__(self, model, mode='empirical', seed=1, *, family='type1', fixed=None):
        if mode not in ('empirical', 'median', 'fixed'):
            raise ValueError('Latency mode must be empirical, median or fixed')
        self.mode, self.random = mode, random.Random(seed)
        self.lists = {'type1_first': model['type1']['burst_first'], 'type1_rest': model['type1']['burst_rest'],
                      'stop_first': model['stop']['burst_first'] or model['type1']['burst_first'],
                      'stop_rest': model['stop']['burst_rest'] or model['type1']['burst_rest'],
                      'single': model['single']}
        self.medians = {key: int(quantile(values, .5)) for key, values in self.lists.items()}
        self.fixed = dict(self.medians)
        if fixed:
            self.fixed.update(fixed)

    def __call__(self, key):
        if self.mode == 'median':
            return self.medians[key]
        if self.mode == 'fixed':
            return self.fixed[key]
        return self.random.choice(self.lists[key])


def load(path=DEFAULT_PATH):
    model = json.loads(Path(path).read_text())
    if model.get('schema') != SCHEMA:
        raise ValueError('Harness latency model schema differs')
    return model


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', nargs='+')
    parser.add_argument('--output', default=str(DEFAULT_PATH))
    args = parser.parse_args(argv)
    model = extract(args.reports)
    Path(args.output).write_text(json.dumps(model, separators=(',', ':'))+'\n')
    print(json.dumps({'sources': model['sources'], 'summary': model['summary']}, indent=1))


if __name__ == '__main__':
    main()

"""Unchanged parser for the existing six library probes and abort receipts."""
import json

SPECS = {'rational': (48, 24, 18, 6), 'quadratic': (64, 32, 24, 8),
         'integer': (24, 12, 9, 3), 'arc': (96, 48, 36, 12)}


def parse(suite, raw, count):
    if suite == 'broad':
        rows = []
        for line in raw.splitlines():
            if line.startswith('operation_'):
                name, iterations, nanos, witness = line.split('\t')
                rows.append({'name': name, 'iterations': int(iterations), 'nanos': int(nanos), 'witness': witness})
        assert len(rows) == 231 and len({r['name'] for r in rows}) == 231
        return rows, [{k: v for k, v in r.items() if k != 'nanos'} for r in rows]
    if suite == 'metadata':
        rows = []
        for line in raw.splitlines():
            if 'REGION_COST ' in line:
                case, phase, iterations, nanos, calls, allocated, live, peak = line.split('REGION_COST ', 1)[1].split()
                rows.append({'name': case + '/' + phase, 'iterations': int(iterations), 'nanos': int(nanos),
                             'allocation_calls': int(calls), 'allocated_bytes': int(allocated),
                             'retained_bytes_delta': int(live), 'peak_bytes_delta': int(peak)})
        assert len(rows) == 18 and all(r['iterations'] == count for r in rows)
        return rows, [{k: r[k] for k in ['name', 'iterations']} for r in rows]
    rows = [json.loads(line) for line in raw.splitlines()]
    if suite == 'abort':
        assert len(rows) == 210
        return rows, rows
    total, named, witnesses, fixtures = SPECS[suite]
    assert len(rows) == total and sum('name' in r for r in rows) == named
    assert sum('witness' in r for r in rows) == witnesses and sum('fixture' in r for r in rows) == fixtures
    assert all(r['iterations'] == count for r in rows if 'name' in r)
    return rows, [r for r in rows if 'witness' in r or 'fixture' in r]

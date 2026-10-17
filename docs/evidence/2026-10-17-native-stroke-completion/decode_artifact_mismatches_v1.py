"""Decode retained assertion bytes and compare every JSON field/archive entry."""
import hashlib, io, json, re, zipfile
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'artifact-mismatches-v1'
RAW = W / 'consumer-validation-v1/circuit-all.stdout'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def digest(value):
    return hashlib.sha256(value).hexdigest()

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

def differences(a, b, path='$'):
    if type(a) is not type(b):
        return [{'path': path, 'expected': a, 'actual': b}]
    if isinstance(a, dict):
        out = []
        for key in sorted(a.keys() | b.keys()):
            if key not in a or key not in b:
                out.append({'path': path + '.' + key, 'expected': a.get(key), 'actual': b.get(key)})
            else:
                out.extend(differences(a[key], b[key], path + '.' + key))
        return out
    if isinstance(a, list):
        if len(a) != len(b):
            return [{'path': path, 'expected': a, 'actual': b}]
        return [d for i, (x, y) in enumerate(zip(a, b)) for d in differences(x, y, path + f'[{i}]')]
    return [] if a == b else [{'path': path, 'expected': a, 'actual': b}]

rows = json.loads((W / 'consumer-validation-v1/commands.json').read_text())
row = next(r for r in rows if r['name'] == 'circuit-all')
assert row['exit_code'] == 101 and sha(RAW) == row['stdout_sha256']
assert RAW.stat().st_size < 20 * 1024**2
O.mkdir(exist_ok=False)
cases, name = {}, None
for line in RAW.read_text().splitlines():
    match = re.match(r"thread '([^']+)' .* panicked at", line)
    if match:
        name = match[1]
        assert name not in cases
        cases[name] = {}
    stripped = line.strip()
    for prefix, label in [('left: ', 'actual'), ('right: ', 'expected')]:
        if name and stripped.startswith(prefix):
            values = json.loads(stripped[len(prefix):])
            assert all(type(v) == int and 0 <= v < 256 for v in values)
            assert label not in cases[name]
            cases[name][label] = bytes(values)
assert len(cases) == 7 and all(set(v) == {'actual', 'expected'} for v in cases.values())
records = {}
for name, pair in cases.items():
    out = O / name
    out.mkdir()
    if name.endswith('_runs_end_to_end_through_hyperdrc'):
        for label, data in pair.items():
            (out / (label + '.json')).write_bytes(data)
        diffs = differences(json.loads(pair['expected']), json.loads(pair['actual']))
        dump(out / 'differences.json', diffs)
        records[name] = {'kind': 'json', 'differences': diffs,
            'sha256': {label: digest(data) for label, data in pair.items()}}
    else:
        assert name == 'default_manufacturing_release_is_unsigned_deterministic_and_self_verifying'
        for label, data in pair.items():
            (out / (label + '.zip')).write_bytes(data)
        archives = {label: zipfile.ZipFile(io.BytesIO(data)) for label, data in pair.items()}
        assert all(z.testzip() is None for z in archives.values())
        names = {label: z.namelist() for label, z in archives.items()}
        entries = []
        for entry in sorted(set(names['expected']) | set(names['actual'])):
            data = {label: z.read(entry) if entry in z.namelist() else None for label, z in archives.items()}
            record = {'path': entry, 'identical': data['expected'] == data['actual'],
                'sha256': {label: digest(value) if value is not None else None for label, value in data.items()},
                'bytes': {label: len(value) if value is not None else None for label, value in data.items()}}
            if not record['identical']:
                for label, value in data.items():
                    if value is not None:
                        p = out / label / entry
                        assert '..' not in Path(entry).parts and not Path(entry).is_absolute()
                        p.parent.mkdir(parents=True, exist_ok=True)
                        p.write_bytes(value)
                if entry.endswith('.json') and all(v is not None for v in data.values()):
                    record['json_differences'] = differences(json.loads(data['expected']), json.loads(data['actual']))
            entries.append(record)
        records[name] = {'kind': 'zip', 'entry_order_identical': names['expected'] == names['actual'],
            'entries': entries, 'changed_entries': [r['path'] for r in entries if not r['identical']]}
record = {'complete': True, 'runner_sha256': sha(__file__), 'raw_stdout_sha256': sha(RAW),
    'original_failed_command': row, 'cases': records, 'snapshots_updated': False,
    'runtime_executed': False, 'main_sources_modified': False}
dump(O / 'result.json', record)
for name, item in records.items():
    print(name)
    if item['kind'] == 'json':
        print(json.dumps(item['differences'])[:5000])
    else:
        print('changed entries', item['changed_entries'])
        for entry in item['entries']:
            if not entry['identical']:
                print(entry['path'], json.dumps(entry.get('json_differences', entry))[:5000])

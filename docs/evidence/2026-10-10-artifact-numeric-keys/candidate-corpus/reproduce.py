"""Exact Fraction oracle for the public production-artifact readiness boundary.

Run serially, without source edits, Cargo work, or competing measurements.
Source and library identities are checked before and after every replay.
"""
from collections import Counter
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PHASE = sys.argv[1]
RECORD = Path(sys.argv[2]).resolve()
OUT = EVIDENCE / PHASE
OUT.mkdir()
WORK = ROOT / ('target/artifact-numeric-keys-' + PHASE)
WORK.mkdir(exist_ok=True)
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': sha(p)}
record = read(RECORD)
source, target = Path(record['source_directory']), Path(record['target_directory'])
guards = []
for kind, base in [('source', source), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert sha(path) == record[kind + '_hashes_file_sha256']
    guards.append((base, json.loads(gzip.decompress(path.read_bytes()))))
library = target / 'release/libhyperdrc.rlib'
matched = [p for p in (target / 'release/deps').glob('libhyperdrc-*.rlib') if sha(p) == sha(library)]
assert len(matched) == 1
fingerprints = target / 'release/.fingerprint'
metadata = read(fingerprints / matched[0].stem.removeprefix('lib') / 'lib-hyperdrc.json')
identifier = next(row[3] for row in metadata['deps'] if row[1] == 'serde_json')
matched_json = [p for p in (target / 'release/deps').glob('libserde_json-*.rlib') if int.from_bytes(bytes.fromhex((fingerprints / p.stem.removeprefix('lib') / 'lib-serde_json').read_text()), 'little') == identifier]
assert len(matched_json) == 1
libraries = {'hyperdrc': library, 'serde_json': matched_json[0]}
library_info = {str(p): info(p) for p in libraries.values()}
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'probe.rs', OUT / 'probe.rs')
shutil.copyfile(source / 'crates/hyperdrc/src/checks/artifacts.rs', OUT / 'artifacts.rs')
frozen = {p: info(p) for p in OUT.iterdir()}
commands = []


def verify():
    assert all(sha(base / name) == expected for base, values in guards for name, expected in values.items())
    assert all(info(Path(p)) == expected for p, expected in library_info.items())
    assert all(info(p) == expected for p, expected in frozen.items())


def limit():
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command, stdin=None, seconds=120, capped=False):
    verify()
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', str(seconds), *map(str, command)], cwd=ROOT, input=stdin, text=True, stdout=stdout, stderr=stderr, preexec_fn=limit if capped else None, check=False)
    commands.append({'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds, 'address_space_cap_mib': 512 if capped else None})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    return result.returncode, (OUT / (name + '.stdout')).read_text()


command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', OUT / 'probe.rs', '-L', 'dependency=' + str(target / 'release/deps')]
for name, path in libraries.items():
    command += ['--extern', name + '=' + str(path)]
assert run('build', command + ['-o', WORK / 'probe'])[0] == 0
frozen[WORK / 'probe'] = info(WORK / 'probe')
print('Probe built; replay begins.', flush=True)
rows = []
prefix = '# units mm; origin aux origin; rotation convention clockwise degrees\nRef,X,Y,Rotation,Side,Value,Footprint\n'


def add(family, left, right, axis='x', same_side=True):
    strip = lambda value: value.removesuffix('mm').removesuffix('deg').strip()
    a, b = Fraction(strip(left)), Fraction(strip(right))
    if family == 'placement':
        one = (left, '2') if axis == 'x' else ('2', left)
        two = (right, '2') if axis == 'x' else ('2', right)
        centroid = prefix + f'R1,{one[0]},{one[1]},0,Top,10k,0603\nR2,{two[0]},{two[1]},0,{"Top" if same_side else "Bottom"},10k,0603\n'
        bom = 'Ref,Qty,MPN\nR1,1,RC0603\nR2,1,RC0603\n'
        expected = a == b and same_side
        needle = 'centroid places multiple references at '
    elif family == 'rotation-parity':
        bom = f'Ref,Qty,MPN,Rotation\nR1,1,RC0603,{left}\n'
        centroid = prefix + f'R1,0,0,{right},Top,10k,0603\n'
        expected = a % 360 != b % 360
        needle = 'BOM rotation for R1 is '
    elif family == 'rotation-conflict':
        bom = 'Ref,Qty,MPN\nR1,1,RC0603\n'
        centroid = prefix + f'R1,0,0,{left},Top,10k,0603\nR1,1,0,{right},Top,10k,0603\n'
        expected = a % 360 != b % 360
        needle = 'centroid reference R1 appears with multiple rotations: '
    else:
        assert family == 'polarized'
        bom = 'Ref,Qty,MPN,Value,Footprint,Polarity\nD1,1,LED0603,LED,0603 LED,Cathode mark\nD2,1,LED0603,LED,0603 LED,Cathode mark\n'
        centroid = prefix + f'D1,0,0,{left},Top,LED,0603 LED\nD2,1,0,{right},Top,LED,0603 LED\n'
        expected = a % 360 != b % 360
        needle = 'at multiple centroid rotations ('
    rows.append({'id': len(rows), 'family': family, 'left': left, 'right': right, 'axis': axis, 'same_side': same_side, 'expected': expected, 'needle': needle, 'bom': bom, 'centroid': centroid})


pairs = [('0.000111', '0.000114'), ('5', '5.00001'), ('0.1', '1/10'), ('+0.0', '-0'), ('1e-9', '0.000000001'), ('1e-400', '2e-400'), ('-0.00001', '0.00001'), ('1e20', '100000000000000000001'), ('1/3', '2/6'), ('0.5mm', '1/2')]
for left, right in pairs:
    for axis in ['x', 'y']:
        add('placement', left, right, axis)
    add('placement', left, right, same_side=False)
rotation_pairs = [('10.00011', '10.00014'), ('0.0001', '360.0002'), ('-180', '180'), ('-720', '0'), ('360', '-0'), ('10.00011', '370.00011'), ('1e-400', '2e-400'), ('-1e-400', '0'), ('1/3', '1081/3'), ('90deg', '450'), ('+0.0', '-0')]
for left, right in rotation_pairs:
    for family in ['rotation-parity', 'rotation-conflict', 'polarized']:
        add(family, left, right)
for numerator in range(-8, 9):
    for denominator in [3, 7, 10000, 10000000]:
        value = Fraction(numerator, denominator)
        for other in [value, value + Fraction(1, 10**8), value + 360]:
            for family in ['placement', 'rotation-parity', 'rotation-conflict', 'polarized']:
                add(family, str(value), str(other))
(OUT / 'inputs.json').write_text(json.dumps(rows, indent=2) + '\n')
trials = []
for order in ['forward', 'reverse']:
    selected = rows if order == 'forward' else list(reversed(rows))
    stdin = ''.join(json.dumps(row) + '\n' for row in selected)
    (OUT / (order + '.stdin')).write_text(stdin)
    code, output = run(order, [WORK / 'probe'], stdin, capped=True)
    assert code == 0
    receipts = {row['id']: row for row in map(json.loads, output.splitlines())}
    assert set(receipts) == {row['id'] for row in rows}
    differences = []
    for row in rows:
        messages = [v.get('message', '') for v in receipts[row['id']]['violations']]
        actual = any(row['needle'] in message for message in messages)
        if actual != row['expected']:
            differences.append({'id': row['id'], 'family': row['family'], 'expected': row['expected'], 'actual': actual})
    trials.append({'order': order, 'cases': len(rows), 'differences': differences, 'by_family': dict(Counter(row['family'] for row in differences))})
    if order == 'forward':
        forward = receipts
    else:
        assert receipts == forward
    print(order, len(rows), 'cases;', len(differences), 'oracle differences.', flush=True)
resources = []
for token in ['not-a-number', 'NaN', '1/0', '1e1000000000', '1e-1000000000', '1e999999999999999999999999x']:
    for field in ['x', 'rotation', 'bom-rotation']:
        row = {'id': field + ':' + token, 'bom': f'Ref,Qty,MPN,Rotation\nR1,1,RC0603,{token if field == "bom-rotation" else "0"}\n', 'centroid': prefix + f'R1,{token if field == "x" else "0"},0,{token if field == "rotation" else "0"},Top,10k,0603\n'}
        name = 'resource-' + str(len(resources))
        stdin = json.dumps(row) + '\n'
        (OUT / (name + '.stdin')).write_text(stdin)
        code, output = run(name, [WORK / 'probe'], stdin, seconds=3, capped=True)
        resources.append({'name': name, 'field': field, 'token': token, 'exit_code': code, 'completed': bool(output.strip())})
summary = {'source_record': str(RECORD), 'source_record_sha256': sha(RECORD), 'libraries': library_info, 'probe': info(WORK / 'probe'), 'trials': trials, 'resources': resources, 'guards_unchanged': True, 'commands': commands}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.iterdir()) if p.is_file()}
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Replay complete.', flush=True)

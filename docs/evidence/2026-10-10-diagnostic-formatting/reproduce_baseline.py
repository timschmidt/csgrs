"""Audit admitted log/display consumers in isolated processes, preserving failures."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PREVIOUS = EVIDENCE.parent / '2026-10-10-hyperdrc-rule-export/candidate-v2'
OUT = EVIDENCE / 'baseline'
WORK = ROOT / 'target/diagnostic-formatting-baseline'
read = lambda p: json.loads(p.read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}
record = read(PREVIOUS / 'sources.json')
validation = read(PREVIOUS / 'validation.json')
SOURCE = Path(record['source_directory'])
TARGET = Path(record['target_directory'])
maps = []
for kind, directory in [('source', SOURCE), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert digest(path) == record[kind + '_hashes_file_sha256']
    maps.append((directory, json.loads(gzip.decompress(path.read_bytes()))))
library = TARGET / 'release/libhyperdrc.rlib'
assert info(library) == validation['libraries'][str(library)]
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(EVIDENCE / 'consumer_probe.rs', OUT / 'probe.rs')
shutil.copyfile(__file__, OUT / 'reproduce.py')
frozen = {name: digest(OUT / name) for name in ['probe.rs', 'reproduce.py']}
linked = WORK / 'libhyperdrc_admitted.rlib'
shutil.copyfile(library, linked)
libraries = {library: info(library), linked: info(linked)}
matches = [p for p in (TARGET / 'release/deps').glob('libhyperdrc-*.rlib') if digest(p) == digest(library)]
assert len(matches) == 1
fingerprints = TARGET / 'release/.fingerprint'
crate_id = matches[0].stem.removeprefix('lib')
metadata = read(fingerprints / crate_id / 'lib-hyperdrc.json')
dep_id = next(row[3] for row in metadata['deps'] if row[1] == 'hyperlattice')
lattice = [p for p in (TARGET / 'release/deps').glob('libhyperlattice-*.rlib') if int.from_bytes(bytes.fromhex((fingerprints / p.stem.removeprefix('lib') / 'lib-hyperlattice').read_text()), 'little') == dep_id]
assert len(lattice) == 1
logs = list((TARGET / 'release/deps').glob('liblog-*.rlib'))
assert len(logs) == 1
libraries |= {p: info(p) for p in [lattice[0], logs[0]]}
commands, rows = [], []


def verify():
    assert all(digest(directory / name) == expected for directory, values in maps for name, expected in values.items())
    assert all(info(p) == expected for p, expected in libraries.items())
    assert all(digest(OUT / name) == expected for name, expected in frozen.items())


def limit_probe():
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))


def run(name, command, seconds=60, probe=False):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', str(seconds), *map(str, command)], cwd=ROOT, stdout=stdout, stderr=stderr, preexec_fn=limit_probe if probe else None, check=False)
    row = {'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds, 'probe_address_space_cap_mib': 512 if probe else None}
    commands.append(row)
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    return row


assert run('format', ['rustfmt', '--edition', '2024', '--check', OUT / 'probe.rs'])['exit_code'] == 0
binary = WORK / 'probe'
assert run('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', OUT / 'probe.rs', '-L', 'dependency=' + str(TARGET / 'release/deps'), '--extern', 'hyperdrc=' + str(linked), '--extern', 'hyperlattice=' + str(lattice[0]), '--extern', 'log=' + str(logs[0]), '-o', binary])['exit_code'] == 0
families = ['0.15', '0', '1e-40', 'pi', 'unresolved_zero', 'aborted_pi']
for operation in ['switch', 'inductor', 'via', 'component', 'complex', 'vector']:
    for index, family in enumerate(families):
        for preparation in ['fresh', 'prepared']:
            for level in (['off', 'trace'] if operation not in ['complex', 'vector'] else ['off']):
                name = f'{operation}-{index}-{preparation}-{level}'
                result = run(name, [binary, operation, family, preparation, level], seconds=3, probe=True)
                rows.append({'operation': operation, 'family': family, 'preparation': preparation, 'log_level': level, 'exit_code': result['exit_code'], 'stdout': (OUT / (name + '.stdout')).read_text(), 'stderr': (OUT / (name + '.stderr')).read_text(), 'wall_seconds': result['wall_seconds']})
                (OUT / 'rows.json').write_text(json.dumps(rows, indent=2) + '\n')
verify()
summary = {'scope': 'Admitted production libraries with unchanged source guards. Empty-board readiness calls compare tracing off/on; aggregate alternate displays use the same scalar families. External cutoffs are observations, not library limits.', 'source_record': str(PREVIOUS / 'sources.json'), 'source_record_sha256': digest(PREVIOUS / 'sources.json'), 'all_source_library_and_probe_hashes_unchanged': True, 'libraries': {str(p): value for p, value in libraries.items()}, 'binary': {'path': str(binary), **info(binary)}, 'rows': rows, 'commands': commands, 'cutoffs': sum(row['exit_code'] == 124 for row in rows), 'cases': len(rows)}
summary['artifacts'] = {str(p.relative_to(EVIDENCE)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
summary['scripts'] = {name: info(EVIDENCE / name) for name in ['consumer_probe.rs', 'reproduce_baseline.py']}
(EVIDENCE / 'baseline.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Diagnostic baseline complete:', len(rows), 'cases;', summary['cutoffs'], 'external cutoffs.', flush=True)
